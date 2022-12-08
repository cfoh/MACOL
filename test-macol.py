'''
This program simulates a cluster of mmWave BSs serving vehicles in a highway (M26 in UK) scenario.
We introduce Multi-Agent Context Learning (MACOL) to manage transmission interference.

The program requires Pymosim v0.8.8 platform to run. Download Pymosim from:
- source code (to release to public soon): https://github.com/cfoh/pymosim
- online documentation: https://cfoh.github.io/pymosim-doc/start.html
'''

from abc import abstractclassmethod
from re import X
import wx
import operator
import argparse
import random
import math
from datetime import datetime
from argparse import Namespace, ArgumentParser

## pymosim packages
from sim.simulation import World
from sim.loc import ScreenXY as XY
from sim.scenario import BaseScenario
from sim.event import Event
from node.node import BaseNode
from node.mobility import Stationary, StaticPath
from comm.transceiver import Transceiver, TransceiverDir
from comm.channel import DiscModel
import node.type as NodeType

####################################################################
## setup options here:
##
class GlobalVariable: pass
gvar = GlobalVariable()
gvar.num_cars = 20 # number of cars to simulate
#gvar.algo = { "option":1 } # option 1: best SNR
#gvar.algo = { "option":2, "explore-first":600, "epsilon":0.05 } # option 2: MACOL
gvar.algo = { "option":2, "explore-first":2*60, "epsilon":0.05 } # option 2: MACOL, different exploration time

gvar.option = {} # other settings
gvar.option["show displacement CDF"] = False

####################################################################
## Helper
####################################################################

class DebugPrint:
    def print1(self, *args, **kw): # show info (debug level 1)
        #print(*args, **kw) # replace with `pass` to disable debug printing
        pass
    def print2(self, *args, **kw): # show detailed statistics (debug level 2)
        #print(*args, **kw)  # replace with `pass` to disable debug printing
        pass
    def print3(self, *args, **kw): # report periodic outcomes (debug level 3)
        print(*args, **kw)  # replace with `pass` to disable debug printing

class Writer:
    '''
    This is a helper class to write traces into a file.
    '''
    def __init__(self, session_name):
        self.session_name = session_name
        self.is_first = True
    def write(self, data:str):
        with open(self.session_name, "a") as file:
            file.write(data+"\n")


####################################################################
## Nodes
####################################################################

class MySector(BaseNode):
    '''
    MySector: This is a base station in the VANET sim world implementing
    ideal sector radiation (option 1) or mmWave beamforming (option 2)
    '''
    def __init__(self, simworld, id, loc, freq, channel, sector_width, sector_dir):
        super().__init__(simworld, id, node_type=NodeType.BaseStation(self))

        ## create transceiver
        self.transceiver = TransceiverDir(self,freq,channel,sector_width,sector_dir)

        ## setup the sector
        self.set_transceiver(self.transceiver)
        self.set_mobility(Stationary(loc))

        ## initialize properties
        self.serving_node = None

        ## connection stats for overall total, current period, current connection
        ## - duration: connection duration
        ## - interference_free: duration only counting interference free
        ## - conn_count: number of connections
        self.total_duration = 0           # overall connection duration
        self.total_interference_free = 0  # overall interference free duration
        self.period_duration = 0          # connection duration for current period
        self.period_interference_free = 0 # interference free duration for current period
        self.period_conn_count = 0        # number of connections for current period
        self.serving_duration = 0          # service duration for current connection
        self.serving_interference_free = 0 # interference free duration for current connection

        ## MAB releated properties
        self.neighbour_list = [] # should carry sector objects
        self.MAB_context = None
        self.MAB_backoff = (0,0) # (start_time, duration)

    def associate_vehicle(self,node,time):
        self.serving_node = node
        self.serving_node.associated_sector = self
        self.serving_duration = 0          # connection duration (both interference & inter-free)
        self.serving_interference_free = 0 # interference free duration
        self.period_conn_count += 1        # connection count for this period
        self.serving_node_x,_ = self.serving_node.get("location").get_xy()

    def lost_vehicle(self,time):
        (x,_) = self.serving_node.get("location").get_xy()
        self.serving_displacement = self.serving_node_x - x
        if self.serving_displacement<0:
            self.serving_displacement = -self.serving_displacement
        self.serving_node.associated_sector = None
        self.serving_node = None

    def backoff(self,start_time,duration):
        self.MAB_backoff = (start_time, duration) # activate backoff

    def is_backoff(self,check_time):
        (start_time, duration) = self.MAB_backoff
        return True if (check_time-start_time)<duration else False

    ## show the coverage of this sector
    def show_coverage(self):
        self.clear_drawing() # this is persistent drawing, so need to clear the all first
        if self.serving_node!=None:
            if self.transceiver.get_property("type")=="omni":
                self.draw_circle(self.transceiver.get_property("radius"))
            elif (self.transceiver.get_property("type")=="directional"
                or self.transceiver.get_property("type")=="mmWaveBeam"):
                self.draw_sector(self.transceiver.get_property("radius"),
                                 self.transceiver.get_property("azimuth"),
                                 self.transceiver.get_property("beam width"))


class MyVehicle(BaseNode):
    '''
    MyVehicle: This is a transmitting node in the VANET sim world
    '''
    def __init__(self, simworld, id, freq, channel):
        super().__init__(simworld, id, node_type=NodeType.Vehicle(self))

        ## create transceiver
        self.transceiver = Transceiver(self,freq,channel)
        self.set_transceiver(self.transceiver)

        ## connection status
        self.associated_sector = None
        self.has_interference = False        

        ## connection stats
        class Stats:
            def __init__(self):
                self.disconnected = 0
                self.connected = 0
                self.interfered = 0
        self.stats = Stats()

    ## draw a line to the associated sector, if any
    def show_connection(self):
        self.clear_drawing() # this is persistent drawing, so need to clear the all first
        if self.associated_sector!=None:
            if self.has_interference:
                # vehicle with a sector & has interference
                self.set_color(wx.BLACK)
                self.draw_line(self.associated_sector,pen = wx.Pen(wx.BLACK,1,style=wx.PENSTYLE_SHORT_DASH))
            else:  
                # vehicle with a sector & has no interference
                self.draw_line(self.associated_sector,pen = wx.Pen(wx.BLUE,2,style=wx.PENSTYLE_SOLID))
                self.set_color(wx.BLUE)
        else:
            # vehicle without a sector association
            self.set_color(wx.RED)

####################################################################
## Algorithm - Base class
####################################################################

class BaseAlgorithm(DebugPrint):
    '''
    This is the base class for an algorithm implementation. Only the methods
    defined here are exposed to the user simulation.
    '''

    def __init__(self, sector_list):
        '''Calling this constructor is unnecessary. All subclasses must 
        define a meaningful `name`.'''
        self.name = "This is a base class with no implementation"
        self.conn_info = {}
        for sector in sector_list: # each sector is a ML agent
            self.conn_info[sector] = []  # for connection info

    def finish(self):
        '''This method will be called at the end of the simulation.'''
        for sector in self.conn_info:
            if len(self.conn_info[sector])==0: continue
            print("For sector %s:"%sector.id)
            if gvar.option["show displacement CDF"]:
                print(" service_period, total_period, displacement, intr-free-disp")
            total_displacement = 0
            total_count = 0
            for (service_time,duration,displacement) in self.conn_info[sector]:
                if gvar.option["show displacement CDF"]:
                    print(" %1.2f, %1.2f, %1.2f, %1.2f"%
                            (service_time,duration,displacement,(service_time/duration)*displacement))
                total_displacement += (service_time/duration)*displacement
                total_count += 1
            print("- average vehicle service displacement = %1.2f"%(total_displacement/total_count))
                

    @abstractclassmethod
    def execute(self,sim_time,duration,all_vehicles,all_sectors):
        '''This method executes the vehicle selection.'''
        pass


####################################################################
## Algorithm - Base station centric association
## Base station (BS) chooses a vehicle to connect
####################################################################

class BSCentric(BaseAlgorithm):

    def __init__(self, sector_list):
        super().__init__(sector_list)
        self.name = "Base Station centric, selecting highest SNR"

    def execute(self,sim_time,duration,all_vehicles,all_sectors):
        '''This method executes the vehicle selection.'''

        ## check and update current association
        for sector in all_sectors:
            if sector.serving_node is not None:
                beacon = sector.transceiver.create_signal()
                if sector.serving_node not in dict(sector.transceiver.broadcast(beacon)):
                    sector.lost_vehicle(sim_time)
                    if True: # - set True to collect the service time of a specified beam,
                    #if False: #   or False to disable the collection
                        if sector.id=="BS-1.0": # mid south BS, north pointing beam
                            self.conn_info[sector].append(
                                (sector.serving_interference_free,
                                 sector.serving_duration,
                                 sector.serving_displacement))

        ## check for new association
        for sector in all_sectors:

            if sector.serving_node is not None: continue # skip if already in service

            vehicle_max = None # vehicle with best SNR
            detection_list = []
            beacon = sector.transceiver.create_signal()
            reply_list = sector.transceiver.broadcast(beacon)
            for (node,_) in reply_list:

                ## step 1: check that the reachable node is an unassociated vehicle
                if node.type!=NodeType.Vehicle: continue # skip if not Vehicle Type
                if node.associated_sector!=None: continue # skip if already associated

                ## step 2: vehicle replies beacon for sector to obtain the signal quality
                beacon_reply = node.transceiver.create_signal()
                recv_signal = node.transceiver.unicast(beacon_reply,sector)
                if recv_signal is None: continue # skip if failed, likely not in coverage

                ## step 3: append to the detection list
                detection_list.append((node,recv_signal.quality))

            ## step 4: associate with the vehicle that has the strongest SNR, if exists
            if len(detection_list)!=0:
                vehicle_max = max(detection_list,key=operator.itemgetter(1))[0]
            if vehicle_max is not None:
                sector.associate_vehicle(vehicle_max,sim_time)
                self.print1("at t=%1.2f, %s now serves %s"%(sim_time,sector.id,vehicle_max.id))


####################################################################
## Contextual Learning Algorithm
## Base station centric using context learning 
####################################################################

class CLMAB(BaseAlgorithm):

    def __init__(self, sector_list, exploration_time):
        super().__init__(sector_list)
        self.name = "Context Learning MAB"
        self.exploration_rate = 1.0
        self.exploration_time = exploration_time
        self.is_exploration_over = False
        self.q_value = {}
        self.q_count = {}
        self.conn_info = {}
        self.sector_list = sector_list
        for sector in sector_list: # each sector is a ML agent
            self.q_value[sector] = {} # for MAB
            self.q_count[sector] = {}
            self.conn_info[sector] = []   # for connection info

    def get_current_context(self, sector):
        '''Return the current context for the given `sector`.
        '''
        context = "["
        for neighbour in sector.neighbour_list:
            context += "1" if neighbour.serving_node!=None else "0"
        return context+"]"

    def get_threshold(self, sector):
        '''Calculate the threshold reward value for the `sector`.
        The threshold is used to classify whether a context is interfering 
        or non-interfering. Here, we simply use the average of rewards across
        all contexts for the threshold.
        '''
        if len(self.q_value[sector])==0: return 0
        sum = 0
        for context in self.q_value[sector]:
            sum += self.q_value[sector][context]
        return sum/len(self.q_value[sector])

    def update_reward(self,sector,context,reward):
        '''Update reward for a context of a sector into the table. 
        `q_count[sector][context]` is incremented by 1, and 
        `q_value[sector][context]` is the updated average reward.
        '''
        if context==None: print("Context is None. Check the code!!!")
        if context not in self.q_value[sector]:
            self.q_value[sector][context] = 0
            self.q_count[sector][context] = 0
        value = self.q_value[sector][context]*self.q_count[sector][context] + reward
        self.q_count[sector][context] += 1
        self.q_value[sector][context] = value / self.q_count[sector][context]

    def get_reward(self,sector,context):
        '''Get the average reward for a context of a sector from the table. 
        '''
        if context not in self.q_value[sector]:
            return 0
        return self.q_value[sector][context]

    def execute(self,sim_time,duration,all_vehicles,all_sectors):
        '''This method executes the vehicle selection.
        '''

        ## exploration or exploitation
        if sim_time<self.exploration_time: # exploration first
            self.exploration_rate = 1.0 # full exploration
        else:
            if not self.is_exploration_over:
                #if True: # - set True to show how many times each sector has explored
                if False: #   an option (or pulled an arm) during the exploration
                    for sector in self.q_count:
                        print("For sector %s, "%sector.id, end='')
                        overall_count = 0
                        for context in self.q_count[sector]:
                            overall_count += self.q_count[sector][context]
                        print("overall q_count = %d"%overall_count)
                    print("Exploration is over")
                self.is_exploration_over = True
            self.exploration_rate = gvar.algo["epsilon"] # exploration rate

        '''
        ## show the evolution of the ML agent internal learning
        ## use this to see further details into ML agent
        if True:   # - set True to enable the print of the picked sector
        #if False: #   or False to skip
            picked_sector = self.sector_list[5] # pick one sector
            print("Sector %s (context, value):"%picked_sector.id)
            for context in self.q_value[picked_sector]:
                print("  (%s, %1.2f) "%(context, self.q_value[picked_sector][context]), end='')
            print()
        '''

        ## check and update current association
        sector_lost_list = [] # sector that just lost a connection
        for sector in all_sectors:
            if sector.serving_node is not None:
                beacon = sector.transceiver.create_signal()
                if sector.serving_node not in dict(sector.transceiver.broadcast(beacon)):
                    sector_lost_list.append((sector,sector.serving_node))
                    sector.lost_vehicle(sim_time)

        ## connection just lost, so update reward
        for (sector,vehicle) in sector_lost_list:
            if sector.serving_duration==0:  # skip 0 duration, very rare, happened when vehicle got 
                pass                        # connected & moved out of sector immediately
            else: 
                the_reward = sector.serving_interference_free / sector.serving_duration
                self.update_reward(sector,sector.MAB_context,the_reward)
                if True: # - set True to collect the service time of a specified beam,
                #if False: #   or False to disable the collection
                    if self.is_exploration_over:
                        if sector.id=="BS-1.0": # mid south BS, north pointing beam
                            self.conn_info[sector].append(
                                (sector.serving_interference_free,
                                 sector.serving_duration,
                                 sector.serving_displacement))

        ## check for new association
        for sector in all_sectors:

            if sector.serving_node!=None: continue # skip if already in service
            if sector.is_backoff(sim_time): continue # skip if in backoff mode

            vehicle_to_select = None # vehicle to select
            detection_list = []
            beacon = sector.transceiver.create_signal()
            reply_list = sector.transceiver.broadcast(beacon)
            for (node,_) in reply_list:

                ## step 1: check that the reachable node is an unassociated vehicle
                if node.type!=NodeType.Vehicle: continue # skip if not Vehicle Type
                if node.associated_sector!=None: continue # skip if already associated

                ## step 2: vehicle replies beacon for sector to obtain the signal quality
                beacon_reply = node.transceiver.create_signal()
                recv_signal = node.transceiver.unicast(beacon_reply,sector)
                if recv_signal is None: continue # skip if failed, likely not in coverage

                ## step 3: append to the detection list
                detection_list.append((node,recv_signal.quality))

            ## step 4: pick a random vehicle to associate with, if exists
            if len(detection_list)!=0:
                vehicle_to_select = random.choice(detection_list)[0]
            if vehicle_to_select is not None:
                ## exploration or exploitation?
                this_context = self.get_current_context(sector)
                if random.random()<self.exploration_rate:
                    to_serve = True # due to exploration
                else:
                    this_reward = self.get_reward(sector,this_context)
                    this_threshold = self.get_threshold(sector)
                    to_serve = True if this_reward==0 or this_reward>this_threshold else False
                    if this_reward!=0:
                        if to_serve:
                            self.print1("  -at t=%1.2f, %s exploits service, as %1.2f>%1.2f"
                                            %(sim_time,sector.id,this_reward,this_threshold))
                        else:
                            self.print1("  -at t=%1.2f, %s skips, as %1.2f<%1.2f"
                                            %(sim_time,sector.id,this_reward,this_threshold))
                ## serve or backoff?
                if to_serve:
                    sector.associate_vehicle(vehicle_to_select,sim_time)
                    sector.MAB_context = this_context
                    self.print1("  -at t=%1.2f, %s now serves %s"%(sim_time,sector.id,vehicle_to_select.id))
                else:
                    sector.backoff(start_time=sim_time,duration=this_threshold)
                    self.print1("  -at t=%1.2f, %s skips service"%(sim_time,sector.id))


####################################################################
## Scenario
####################################################################

class MyScenario(BaseScenario,DebugPrint):
    '''
    MyScenario: This is my scenario
    '''

    ## ------------------------------------------------------------
    ## This method will be called before the start of the simulation,
    ## build the simulation world here
    def on_create(self, simworld) -> bool:

        ## create a writer using date/time as the filename
        self.out = Writer("session-%s"%datetime.now().strftime("%y-%m-%d_%H-%M-%S"))

        ## for statistics
        class Stats:
            def __init__(self):
                self.reset()
            def reset(self):
                self.disconnected = 0
                self.connected = 0
                self.interfered = 0
                self.time = 0
        self.total = Stats() # to keep overall results
        self.period = Stats() # to keep last 200s period results
        self.last_sim_time = 0
        self.last_period_time = 0

        ## simulation variables
        ## map resolution is 1 pixel/meter
        self.simworld = simworld
        if self.simworld.is_animation_shown():
            bitmap = wx.Bitmap()
            if bitmap.LoadFile("M26.png"):
                self.set_background(bitmap,-500,0)
            else:
                print("Error loading bitmap file, no background is applied.")
        self.set_name("A busy highway (M26)")

        ## configure transceiver property
        beam_radius = 80 # meter
        beam_width = 60  # degree

        ## create channel for signal propagation model
        carrier_freq = 28 # GHz
        ch_disc = DiscModel(beam_radius)
        channel_model = ch_disc

        ## create some nodes on the north and south sides of the highway
        ## all 18 beams at 6 sites for the highway scenario
        ##   (3)   (4)     (5)    <-- north-side beams
        ##   ==================   <-- highway edge
        ##     > >     >          <-- 3 lanes (east moving)
        ##      <     <    <      <-- 3 lanes (west moving)
        ##   ==================   <-- highway edge
        ##   (0)    (1)     (2)   <-- south-side beams
        bs_locs = [XY(100,260),XY(220,260),XY(360,260)] # south locations
        bs_locs += [XY(90,180),XY(210,180),XY(340,180)] # north locations
        self.sector_nodes = []
        for i in [0,1,2]: # south side
            for angle in [360-60, 0, 60]: # 300,0,60 degree (clockwise from north)
                this_id = "BS-%d.%d"%(i,angle)
                this_node = MySector(simworld, this_id, bs_locs[i], 
                                        carrier_freq, channel_model, 
                                        sector_width=beam_width, 
                                        sector_dir=angle)
                self.sector_nodes.append(this_node)
        for i in [3,4,5]: # north side
            for angle in [180+60, 180, 180-60]: # 240,180,120 degree (clockwise from south)
                this_id = "BS-%d.%d"%(i,angle)
                this_node = MySector(simworld, this_id, bs_locs[i], 
                                        carrier_freq, channel_model, 
                                        sector_width=beam_width, 
                                        sector_dir=angle)
                self.sector_nodes.append(this_node)

        ## setup neighbouring sector relationship (this is manually done)
        ## the following is the BS setup:
        ##   [9][10][11]  [12][13][14]  [15][16][17]  <- north BSs
        ##  ====================================HIGHWAY===========
        ##   [0][1][2]       [3][4][5]    [6][7][8]   <- south BSs
        ## [0] faces North-West; [1] faces North; [2] faces North-East,
        ## then repeats for [3],[4],[5], and again for [6][7][8], ...
        neighbouring_sector = { 0: [9,10,1],    # [9] & [10] are neighbours of [0]
                                1: [9,10,11,0,2], # ...
                                2: [10,11,12,1,3],
                                3: [11,12,13,2,4],
                                4: [12,13,14,3,5],
                                5: [13,14,15,4,6],
                                6: [14,15,16,5,7],
                                7: [15,16,17,6,8],
                                8: [16,17,7],
                                9: [0,1,10],
                                10: [0,1,2,9,11],
                                11: [1,2,3,10,12],
                                12: [2,3,4,11,13],
                                13: [3,4,5,12,14],
                                14: [4,5,6,13,15],
                                15: [5,6,7,14,16],
                                16: [6,7,8,15,17],
                                17: [7,8,16] }
        for idx,neighbour_list_idx in neighbouring_sector.items():
            self.print1(f"For sector {self.sector_nodes[idx].id}")
            for neighbour_idx in neighbour_list_idx:
                self.sector_nodes[idx].neighbour_list.append(self.sector_nodes[neighbour_idx])
                self.print1(f" {self.sector_nodes[neighbour_idx].id};", end="")
            self.print1()

        ## setup vehicle info
        self.vehicle_info = {}  # list of [start location, end location]
        y = 208+3; space=4 # assuming 4 meters spacing for each highway lane
        end_point = 480 
        self.vehicle_info["car1"] = [XY(0,y), XY(end_point,y)]; y+=space
        self.vehicle_info["car2"] = [XY(0,y), XY(end_point,y)]; y+=space
        self.vehicle_info["car3"] = [XY(0,y), XY(end_point,y)]; y+=space
        self.vehicle_info["car4"] = [XY(end_point,y), XY(0,y)]; y+=space
        self.vehicle_info["car5"] = [XY(end_point,y), XY(0,y)]; y+=space
        self.vehicle_info["car6"] = [XY(end_point,y), XY(0,y)]; y+=space
        two_cars  = ["car1","car6"]
        four_cars = two_cars + ["car2","car5"]
        six_cars  = four_cars + ["car3","car4"]

        ## create the vehicles on the highway based on above info
        num_cars = gvar.num_cars
        self.vehicles = []
        for info in self.vehicle_info: # minimum 6 cars
            start_loc = self.vehicle_info[info][0]
            end_loc = self.vehicle_info[info][1]
            path = [ (random.uniform(22.3,31.2), end_loc) ] # in m/s, approx. 50-70 mph
            node = MyVehicle(simworld, id=info, freq=carrier_freq, channel=channel_model)
            node.set_mobility(StaticPath(start_loc,path,delay_start=random.uniform(0,5)))
            self.vehicles.append(node)
        cars_delay = 0
        num_cars -= 6
        while num_cars>0:
            cars_delay += 5
            cars_to_add = two_cars if num_cars<=2 else \
                          four_cars if num_cars<=4 else six_cars
            for info in cars_to_add:
                start_loc = self.vehicle_info[info][0]
                end_loc = self.vehicle_info[info][1]
                path = [ (random.uniform(22.3,31.2), end_loc) ] # in m/s, approx. 50-70 mph
                node = MyVehicle(simworld, id=info, freq=carrier_freq, channel=channel_model)
                node.set_mobility(StaticPath(start_loc,path,
                                    delay_start=random.uniform(cars_delay,cars_delay+5)))
                self.vehicles.append(node)
                num_cars -= 1

        # put the following into the class property, needed in `do_restart_node()`
        self.freq = carrier_freq
        self.ch_model = channel_model

        ## create an algorithm
        if gvar.algo["option"]==1:
            self.beam_selection = BSCentric(self.sector_nodes)
        elif gvar.algo["option"]==2:
            self.beam_selection = CLMAB(self.sector_nodes,gvar.algo["explore-first"])
        else:
            print("Error: Wrong algorithm option\n")
            self.out.write("Error: Wrong algorithm option")
            return False

        ## print scenario
        sim_config =  f"Number of vehicles = {len(self.vehicles)}\n"
        sim_config += f"Vehicle selection algorithm: {self.beam_selection.name}\n"
        sim_config += f"===================================\n"
        print("\n"+sim_config)
        self.out.write(sim_config)

        return True

    ## --------------------------------------------------------
    ## This method will be called repeatedly until the simulation
    ## is ended or stopped, perform any simulation action here
    def on_event(self, sim_time, event_obj):

        duration = sim_time - self.last_sim_time
        self.last_sim_time = sim_time

        if event_obj==Event.MOBILITY_END: # a mobile node has finished its mobility?
            self.do_mobility(sim_time,duration,event_obj)
            self.do_restart_node(sim_time,duration,event_obj)
        elif event_obj==Event.SIM_MOBILITY: # mobility progresses a time step?
            self.do_mobility(sim_time,duration,event_obj)
        elif event_obj==Event.SIM_END:  # simulation has ended?
            self.beam_selection.finish()

    ## end of mobility, then create a new vehicle to replace this one
    def do_restart_node(self, sim_time, duration, event_obj):
        this_node = event_obj.get("node") # get the node reaching end of mobility

        ## collect statistics for total
        self.total.connected += this_node.stats.connected
        self.total.disconnected += this_node.stats.disconnected
        self.total.interfered += this_node.stats.interfered
        self.total.time += (this_node.stats.connected + this_node.stats.disconnected 
                            + this_node.stats.interfered)
        self.print2("t=%1.2f, conn=%1.2f (%1.2f%%), no_service=%1.2f (%1.2f%%), interfered=%1.2f (%1.2f%%)"%
                    (sim_time, self.total.connected, 100*self.total.connected/self.total.time,
                               self.total.disconnected, 100*self.total.disconnected/self.total.time,
                               self.total.interfered, 100*self.total.interfered/self.total.time))

        ## collect statistics for last `set_period` second
        set_period = 30 # the period duration to take statistics
        if sim_time>self.last_period_time + set_period:

            period_duration = self.total.time - self.period.time
            self.period.connected = self.total.connected - self.period.connected
            self.period.disconnected = self.total.disconnected - self.period.disconnected
            self.period.interfered = self.total.interfered - self.period.interfered
            self.print3(">>>LAST %d: conn=%1.2f (%1.2f%%), no_service=%1.2f (%1.2f%%), interfered=%1.2f (%1.2f%%)"%
                       (set_period, self.period.connected, 100*self.period.connected/period_duration,
                        self.period.disconnected, 100*self.period.disconnected/period_duration,
                        self.period.interfered, 100*self.period.interfered/period_duration))

            average_period_duration = 0
            average_period_interference_free = 0
            for sector in self.sector_nodes:
                if sector.period_conn_count==0: continue
                sector.period_duration = (sector.total_duration - sector.period_duration) \
                                          / sector.period_conn_count
                sector.period_interference_free = \
                    (sector.total_interference_free - sector.period_interference_free) \
                    / sector.period_conn_count
                average_period_duration += sector.period_duration
                average_period_interference_free += sector.period_interference_free
            average_period_duration /= len(self.sector_nodes)
            average_period_interference_free /= len(self.sector_nodes)
            self.print3(">>>LAST %d: BS conn_duration=%1.2f; int_free=%1.2f (%1.2f%%)"%
                       (set_period,average_period_duration, average_period_interference_free,
                        100*average_period_interference_free/average_period_duration))

            if self.out.is_first:
                self.out.is_first = False
                self.out.write("time, period_duration, conn, no_serv, interfered, conn_duration, int_free_duration, "
                               "conn_perc, outage_perc, interfered_perc")
            self.out.write("%1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f"
                %(sim_time, period_duration,
                  self.period.connected, self.period.disconnected, self.period.interfered,
                  average_period_duration, average_period_interference_free,
                  100*self.period.connected/period_duration,
                  100*self.period.disconnected/period_duration,
                  100*self.period.interfered/period_duration))

            self.print3("t=%1.2f"%sim_time)
            self.last_period_time = sim_time
            self.period.connected = self.total.connected
            self.period.disconnected = self.total.disconnected
            self.period.interfered = self.total.interfered
            self.period.time = self.total.time
            for sector in self.sector_nodes:
                sector.period_duration = sector.total_duration
                sector.period_interference_free = sector.total_interference_free
                sector.period_conn_count = 0

        speed = random.uniform(22.3,31.2)              # new speed
        start_loc = self.vehicle_info[this_node.id][0] # new start location
        end_loc = self.vehicle_info[this_node.id][1]   # new end location
        new_path = [ (speed, end_loc) ]                # build a new path
        new_node = MyVehicle(self.simworld, id=this_node.id, 
                             freq=self.freq, channel=self.ch_model)
        new_node.set_mobility(StaticPath(start_loc=start_loc,path=new_path))
        self.vehicles.append(new_node) # add new node to our list

        self.vehicles.remove(this_node) # remove old node from our list
        this_node.remove_from_simulation() # remove old node from the simulation

    ## Do user simulation here
    ## main task: do BS association if needed
    def do_mobility(self, sim_time, duration, event_obj):

        all_vehicles = self.vehicles    # get all vehicles from our list
        all_sectors = self.sector_nodes # get all sectors from our list

        ## collect stats for each vehicle and sector for the last period
        for vehicle in all_vehicles:
            if vehicle.associated_sector is None:
                vehicle.stats.disconnected += duration
            else:
                if vehicle.has_interference:
                    vehicle.stats.interfered += duration
                else:
                    vehicle.stats.connected += duration
        for sector in all_sectors:
            if sector.serving_node!=None:
                sector.total_duration += duration   # for overall
                sector.serving_duration += duration # for this connection
                if not sector.serving_node.has_interference:
                    sector.total_interference_free += duration   # for overall
                    sector.serving_interference_free += duration # for this connection

        ## run the beam selection algorithm
        self.beam_selection.execute(sim_time,duration,all_vehicles,all_sectors)

        ## check for interference for each vehicle
        for vehicle in all_vehicles:

            ## skip if no BS association, probably outside of BS beam coverage
            if vehicle.associated_sector==None: continue

            ## use hello-beacon to find which other beam also covers this vehicle
            vehicle.has_interference = False
            beacon = vehicle.transceiver.create_signal()
            reply_list = vehicle.transceiver.broadcast(beacon)
            for (beam,signal) in reply_list:

                ## check the bs beam
                if beam.type!=NodeType.BaseStation: continue # skip if not BS
                if beam.serving_node==None: continue         # skip if the bs is not active
                if beam==vehicle.associated_sector: continue # skip if it's the associated BS

                ## at this point, the beam is associated with another vehicle
                ## check that if it can also cover this vehicle
                probe_message = beam.transceiver.create_signal()
                recv_signal = beam.transceiver.unicast(probe_message,vehicle)
                if recv_signal is not None:      # can probe signal reach the vehicle?
                    vehicle.has_interference = True  # if so, set interference to True

        ## draw connectivity & beam coverage on the map
        for vehicle in all_vehicles:
            vehicle.show_connection()
        for beam in all_sectors:
            beam.show_coverage()


####################################################################
## main
####################################################################

if __name__ == "__main__":

    ## command line parameters
    parser: ArgumentParser = argparse.ArgumentParser()
    parser.add_argument("--nodisplay", help="Run in no GUI mode", action="store_true")
    parser.add_argument("--step", help="Mobility step time (in sec)", type=int, default=0.2)
    parser.add_argument("--speed", help="Animation playback speed (x times)", type=float, default=1.0)
    parser.add_argument("--duration", help="Simulation duration (in sec), -1 for non-stop", type=int, default=1)
    args: Namespace = parser.parse_args()

    ## welcome info
    print("A Simple VANET Environment. Press [^C] to quit")
    args.nodisplay = False  # <-- hardcoding no GUI mode
    args.step = 0.1         # <-- hardcoding the mobility step time
    args.speed = 15.0        # <-- hardcoding the animation speed (times)
    args.duration = 1950     # <-- hardcoding the sim duration (sec)

    if args.nodisplay:   print("- simulation will run without animation")
    else:                print("- animation will playback at x%1.2f speed"%args.speed)
    print("- vehicles move a step every %1.2f s in simulation"%args.step)
    if args.duration>0:  print("- simulation will stop at %1.2f s"%args.duration)
    else:                print("- simulation will run non-stop")
    print("")

    ## create, setup and run the simulation
    ## note that to run a simulation, we need to create a 'scenario'
    sim = World()
    sim.config(sim_stop = args.duration, 
               sim_step = args.step, 
               sim_speed = args.speed, 
               display_option = not args.nodisplay, 
               scenario = MyScenario(sim))

    print("Designed for PyMoSim v0.8.8")
    print("Running PyMoSim version v%d.%d.%d"%sim.version())
    sim.run()

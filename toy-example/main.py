'''
This is a toy example illustrating the implementation of MACOL.
The code is tested with Python v3.9.5 and pygame v2.5.2.
'''

import pygame
import sys, math, random, argparse

## global settings
FPS = 60   # frame per second for animation speed
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 400
LEFT,RIGHT    = -1,1 # constants
NUM_VEHICLES_EACH_WAY = 6

###############################################################
# Vehicle
###############################################################

class Vehicle:

    ## static vehicle images
    def create_image(body_color,roof_color):
        image = pygame.Surface((20, 10))
        image.fill(body_color)
        pygame.draw.rect(image, roof_color, ((5,1), (10,8)))
        return image
    image0 = create_image((230, 230, 230), (180, 180, 180))  # white, no connection
    image1 = create_image((50, 50, 150), (0, 100, 255))      # blue, connected
    image2 = create_image((150, 50, 50), (255, 100, 0))      # red, interfered

    def __init__(self, start_pos, direction):
        '''Create a vehicle by specifying it starting position and direction.
        '''
        self.rect = Vehicle.image0.get_rect(center=start_pos)
        self.direction = direction 
        self.wait = random.randint(1,50) # randomize start time
        self.reset()

    def get_xy(self):
        '''Get the (x,y) tuple of the vehicle.
        '''
        return self.rect.center # return a tuple (x,y)

    def reset(self):
        '''Reset vehicle's position to the starting point with a new random speed.
        '''
        self.associated_bs = None
        self.signal_count = 0 # number of BS transmissions received
        self.speed = random.uniform(1,4)
        if self.direction==RIGHT:
            self.rect.left = -self.rect.width
        elif self.direction==LEFT:
            self.rect.right = SCREEN_WIDTH + self.rect.width

    def update(self):
        '''Move the vehicle based on its speed and direction.
        '''
        if self.wait!=0: 
            self.wait -= 1
            return # delayed start, do nothing until wait time is over
        self.rect.move_ip(self.speed*self.direction,0)

        ## check if the vehicle is off-screen, then reset its position and speed
        if self.direction==RIGHT and self.rect.left>SCREEN_WIDTH:
            self.reset()
        elif self.direction==LEFT and self.rect.right<0:
            self.reset()

    def is_connected(self):
        '''Check is this vehicle is connected to a BS.
        '''
        return self.associated_bs is not None

    def draw(self, screen):
        '''Draw this vehicle on the screen.
        '''
        if not self.is_connected():
            image = Vehicle.image0 # no connection
        elif self.signal_count==1:
            image = Vehicle.image1 # connected, no interference
        else:
            image = Vehicle.image2 # exposed to more than 1 BS transmissions, i.e. interfered
        screen.blit(image,self.rect)

###############################################################
# BaseStation
###############################################################

class BaseStation:

    ## static settings
    range = 150

    def __init__(self, pos):
        '''Create a BS by specifying it position.
        '''
        self.x, self.y = pos
        self.serving_vehicle = None
        self.total_serving_duration = 0
        self.total_serving_count = 0

        ## MACOL related properties
        self.MACOL_context = None
        self.MACOL_backoff = (0,0) # (start_backoff_time, backoff_duration)

    def get_xy(self):
        '''Get the (x,y) tuple of the BS.
        '''
        return (self.x,self.y)

    def distance_from(self, other):
        '''Calculate the distance between this BS and another object.
        '''
        x,y = other.get_xy()
        return math.sqrt((self.x-x)**2 + (self.y-y)**2)
    
    def can_reach(self, vehicle):
        '''Test if the input vehicle is within the transmission range of this BS.
        '''
        return self.distance_from(vehicle)<=BaseStation.range

    def associate_vehicle(self, vehicle, time):
        '''Associate the given vehicle with this BS.
        '''
        self.serving_vehicle = vehicle
        self.serving_vehicle.associated_bs = self
        self.serving_duration = 0     # connection duration (both interference & inter-free)
        self.serving_inter_free = 0   # interference free duration

    def lost_vehicle(self, time):
        '''Trigger a lost connection.
        '''
        self.total_serving_duration += self.serving_duration
        self.total_serving_count += 1
        self.serving_vehicle.associated_bs = None
        self.serving_vehicle = None

    def get_average_serving_duration(self):
        '''Calculate the average serving duration for this BS.
        '''
        return self.total_serving_duration / self.total_serving_count

    def backoff(self, start_time, duration):
        '''Activate backoff.
        '''
        self.MACOL_backoff = (start_time, duration) # activate backoff

    def is_backoff(self, check_time):
        '''Check if the BS is in the backoff mode.
        '''
        start_time, duration = self.MACOL_backoff
        return True if (check_time-start_time)<duration else False

    def draw(self, screen):
        '''Draw this BS on the screen.
        '''
        pygame.draw.polygon(screen, color=(30,180,180), 
                            points=[(self.x,self.y),(self.x+8,self.y+20),(self.x-8,self.y+20)])
        ## draw the connection status
        if self.serving_vehicle is not None:
            line_color = (0,0,0) if self.serving_vehicle.signal_count==1 else (255,0,0)
            pygame.draw.line(screen,line_color,self.get_xy(),self.serving_vehicle.get_xy())
            pygame.draw.circle(screen, color=line_color, center=(self.x,self.y), 
                                radius=BaseStation.range, width=2)

###############################################################
# Greedy Approach
###############################################################

class GreedyApproach:

    def __init__(self, bs_list):
        self.name = "Greedy Approach"

    def execute(self, sim_time, vehicle_list, bs_list):
        '''It makes connection decision. Inputs are `vehicle_list` and `bs_list`.
        The algorithm checks the connection status, make connection decision,
        and set the connection assignment and status. After the execution,
        the properties in `bs` and `vehicle` related to connection are set properly.
        '''
        for bs in bs_list:

            ## check and update current connection
            if bs.serving_vehicle is not None:
                if not bs.can_reach(bs.serving_vehicle):
                    bs.lost_vehicle(sim_time) # lost current connection

            ## check for new association
            if bs.serving_vehicle is None:
                reachable_vehicles = []
                for vehicle in vehicle_list:
                    if not vehicle.is_connected() and bs.can_reach(vehicle):
                        distance = bs.distance_from(vehicle)
                        reachable_vehicles.append((vehicle,distance))
                if len(reachable_vehicles)>0:
                    ## for bestSNR option, pick the vehicle nearest to BS for highest SNR
                    best_vehicle = min(reachable_vehicles, key=lambda x: x[1])[0]
                    bs.associate_vehicle(best_vehicle, sim_time) # make connection

###############################################################
# MACOL Solution
###############################################################

class MACOLSolution:

    def __init__(self, bs_list):
        self.name = "MACOL Solution"

        self.epsilon = 0.05 # exploration rate setting
        self.exploration_rate = 1.0  # start with explore first, so set rate to 1.0
        self.exploration_time = 8000
        self.is_full_exploration = True

        ## MACOL setup
        self.q_value = {}  # for Multi-Armed Bandit, the q_table 
        self.q_count = {}  # contains `value` & `count`
        for bs in bs_list:         # in MACOL, each bs is a ML agent, so
            self.q_value[bs] = {}  # create q_table for each ML agent
            self.q_count[bs] = {}  # e.g. for a given `bs`:
                                   #        +-----------+-------+-------+
                                   #        |  context  | value | count |
                                   #        +-----------+-------+-------+
                                   #        | [0,0,0,1] |  25.4 |   4   |
                                   #        | [1,1,0,0] |   4.5 |  12   |
                                   #        | [0,1,0,1] |  18.7 |   7   |
                                   #        +-----------+-------+-------+
                                   #                        ^      ^
                                   #                        |      q_count[bs][context]
                                   #                        +-- q_value[bs][context]
                                   #
                                   # the threshold is based on the average across 
                                   # all contexts, so in this example, it is 16.2

    def get_current_context(self, bs, bs_list):
        '''Return the current context for the given `bs`. The context is a string 
        containing connection status (in "0"/"1") of all other BSs.
        '''
        context = "["
        for neighbor in bs_list:
            if neighbor is bs: continue  # this is own, not a neighbor, so skip
            context += "1" if neighbor.serving_vehicle is not None else "0"
        return context+"]"

    def get_threshold(self, bs):
        '''Calculate the threshold reward value for the `bs`.
        The threshold is used to classify whether a context is interfering 
        or non-interfering. Here, we simply use the average of rewards 
        (i.e. `q_value[bs]`) across all contexts as the threshold value.
        '''
        if len(self.q_value[bs])==0: return 0  # return 0 if q_table is empty
        context_list = list(self.q_value[bs].values())
        return sum(context_list)/len(context_list)

    def update_reward(self, bs, context, reward):
        '''Update reward for a context of a bs into the table. 
        `q_count[bs][context]` is incremented by 1, and 
        `q_value[bs][context]` is the updated average reward.
        '''
        if context not in self.q_value[bs]:
            self.q_value[bs][context] = 0
            self.q_count[bs][context] = 0
        value = self.q_value[bs][context]*self.q_count[bs][context] + reward
        self.q_count[bs][context] += 1
        self.q_value[bs][context] = value / self.q_count[bs][context]

    def get_reward(self, bs, context):
        '''Get the average reward for a context of a bs from the q_table. 
        '''
        if context not in self.q_value[bs]:
            return 0
        return self.q_value[bs][context]

    def execute(self, sim_time, vehicle_list, bs_list):
        '''It makes connection decision. Inputs are `vehicle_list` and `bs_list`.
        The algorithm checks the connection status, make connection decision,
        and set the connection assignment and status. After the execution,
        the properties in `bs` and `vehicle` related to connection are set properly.
        '''

        ## exploration or exploitation
        if self.is_full_exploration:
            if sim_time>self.exploration_time: 
                self.is_full_exploration = False
                self.exploration_rate = self.epsilon

        ## check and update current association
        bs_lost_list = [] # BS that just lost a connection
        for bs in bs_list:
            if bs.serving_vehicle is not None:
                if not bs.can_reach(bs.serving_vehicle):
                    bs_lost_list.append((bs,bs.serving_vehicle))
                    bs.lost_vehicle(sim_time)

        ## connection just lost, so update reward
        for (bs,vehicle) in bs_lost_list:
            the_reward = bs.serving_inter_free
            self.update_reward(bs,bs.MAB_context,the_reward)

        ## check for new association
        for bs in bs_list:

            if bs.serving_vehicle is not None: continue # skip if already in service
            if bs.is_backoff(sim_time): continue # skip if in backoff mode

            ## step 1: establish a list of reachable vehicles
            reachable_vehicles = []
            selected_vehicle = None
            for vehicle in vehicle_list:
                if not vehicle.is_connected() and bs.can_reach(vehicle):
                    reachable_vehicles.append(vehicle)

            ## step 2: pick a random vehicle to associate with, if exists
            if len(reachable_vehicles)!=0:
                selected_vehicle = random.choice(reachable_vehicles)

            ## step 3: use MACOL to judge based on the learned context if this 
            ##         service is transmission-free or transmission-interfered
            if selected_vehicle is not None:
                ## exploration or exploitation?
                this_context = self.get_current_context(bs,bs_list)
                if random.random()<self.exploration_rate:
                    ## in exploration, we're always greedy to maximize learning
                    to_serve = True
                else:
                    ## in exploitation, we check the context and decide, i.e.
                    ## if the expected reward (based on past experience) is higher than
                    ## the threshold, then we serve the vehicle, otherwise do backoff
                    this_reward = self.get_reward(bs,this_context)
                    this_threshold = self.get_threshold(bs)
                    to_serve = True if this_reward==0 or this_reward>this_threshold else False

                ## serve or backoff?
                if to_serve:
                    bs.associate_vehicle(selected_vehicle,sim_time)
                    bs.MAB_context = this_context
                else:
                    bs.backoff(start_time=sim_time,duration=bs.get_average_serving_duration())

###############################################################
# Simulation loop
###############################################################

if __name__ == "__main__":

    ## detect user input `-m` with choices '0' or '1'
    #parser = argparse.ArgumentParser(description="Script description")
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", type=int, choices=[0, 1], required=False, 
                        help="0 for greedy (default), 1 for MACOL")
    args = parser.parse_args()

    ## setup the simulation world
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT))
    pygame.display.set_caption("Highway Communication Simulation")
    clock = pygame.time.Clock()

    ## load background image & zoom in to the right region
    background = pygame.image.load('highway.png').convert_alpha()
    background_aspect_ratio = background.get_width() / background.get_height()
    background = pygame.transform.scale(background, (2000, int(2000/background_aspect_ratio)))

    ## create small cell base stations
    bs_list = []
    for pos in [(300,130),(400,130),(500,130),(350,250),(450,250)]:
        bs_list.append(BaseStation(pos))

    ## create vehicles
    vehicle_list = []
    for i in range(NUM_VEHICLES_EACH_WAY):
        vehicle_list.append(Vehicle((0,180+(i%2)*11),RIGHT))  # moving to right
        vehicle_list.append(Vehicle((SCREEN_WIDTH,204+(i%2)*11),LEFT)) # moving to left

    ## select the algorithm
    if args.m==1:
        algo = MACOLSolution(bs_list)
    else:
        algo = GreedyApproach(bs_list)

    ## simulation loop
    sim_tick = 0
    running = True
    while running:
        for event in pygame.event.get():
            if event.type==pygame.QUIT:
                running = False
        sim_tick += 1

        ## make connection decision
        algo.execute(sim_tick,vehicle_list,bs_list)

        ## check for interference
        for vehicle in vehicle_list:
            if not vehicle.is_connected(): continue # skip those without a connection
            vehicle.signal_count = 0
            for bs in bs_list:
                if bs.serving_vehicle is None: continue # skip any BS not in service
                if bs.can_reach(vehicle):
                    vehicle.signal_count += 1

        ## calculate connection duration: `serving_duration` & `serving_inter_free`
        for bs in bs_list:
            if bs.serving_vehicle is None: continue # skip any BS not in service
            bs.serving_duration += 1   # connection duration (both interference & inter-free)
            if bs.serving_vehicle.signal_count==1:
                bs.serving_inter_free += 1 # interference free duration

        ## draw background
        screen.blit(background,(-550,-100))

        ## draw vehicles
        for vehicle in vehicle_list:
            vehicle.update()
            vehicle.draw(screen)

        ## draw small cell base stations
        for bs in bs_list:
            bs.draw(screen)

        ## show the algorithm method and simulation tick
        text_message = pygame.font.Font(None,36).render(algo.name,True,(0,0,0)) 
        screen.blit(text_message, (20,20))
        text_message = pygame.font.Font(None,30).render(f"time: {sim_tick}",True,(0,0,0)) 
        screen.blit(text_message, (SCREEN_WIDTH-text_message.get_width()-30,20))

        ## render the world
        pygame.display.flip()
        clock.tick(FPS)  # control the speed

    ## exit the program
    pygame.quit()
    sys.exit()
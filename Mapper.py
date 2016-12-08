"""
Example demonstrating how to communicate with Microsoft Robotic Developer
Studio 4 via the Lokarria http interface. 

Author: Erik Billing (billing@cs.umu.se)

Updated by Ola Ringdahl 204-09-11
"""


import httplib, json, time, sys, random
from math import sin,cos,pi,atan2,degrees,radians, sqrt, floor
import matplotlib.pyplot as plt


HEADERS = {"Content-type": "application/json", "Accept": "text/json"}

class UnexpectedResponse(Exception): pass

def postSpeed(angularSpeed,linearSpeed):
    """Sends a speed command to the MRDS server"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    params = json.dumps({'TargetAngularSpeed':angularSpeed,'TargetLinearSpeed':linearSpeed})
    mrds.request('POST','/lokarria/differentialdrive',params,HEADERS)
    response = mrds.getresponse()
    status = response.status
    #response.close()
    if status == 204:
        return response
    else:
        raise UnexpectedResponse(response)

def getLaser():
    """Requests the current laser scan from the MRDS server and parses it into a dict"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/laser/echoes')
    response = mrds.getresponse()
    if (response.status == 200):
        laserData = response.read()
        response.close()
        return json.loads(laserData)
    else:
        return response
    
def getLaserAngles():
    """Requests the current laser properties from the MRDS server and parses it into a dict"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/laser/properties')
    response = mrds.getresponse()
    if (response.status == 200):
        laserData = response.read()
        response.close()
        properties = json.loads(laserData)
        beamCount = int((properties['EndAngle']-properties['StartAngle'])/properties['AngleIncrement'])
        a = properties['StartAngle']#+properties['AngleIncrement']
        angles = []
        while a <= properties['EndAngle']:
            angles.append(a)
            a+=pi/180 #properties['AngleIncrement']
        #angles.append(properties['EndAngle']-properties['AngleIncrement']/2)
        return angles
    else:
        raise UnexpectedResponse(response)

def getPose():
    """Reads the current position and orientation from the MRDS"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/localization')
    response = mrds.getresponse()
    if (response.status == 200):
        poseData = response.read()
        response.close()
        return json.loads(poseData)
    else:
        return UnexpectedResponse(response)

def bearing(q):
    return rotate(q,{'X':1.0,'Y':0.0,"Z":0.0})

def rotate(q,v):
    return vector(qmult(qmult(q,quaternion(v)),conjugate(q)))

def quaternion(v):
    q=v.copy()
    q['W']=0.0;
    return q

def vector(q):
    v={}
    v["X"]=q["X"]
    v["Y"]=q["Y"]
    v["Z"]=q["Z"]
    return v

def conjugate(q):
    qc=q.copy()
    qc["X"]=-q["X"]
    qc["Y"]=-q["Y"]
    qc["Z"]=-q["Z"]
    return qc

def qmult(q1,q2):
    q={}
    q["W"]=q1["W"]*q2["W"]-q1["X"]*q2["X"]-q1["Y"]*q2["Y"]-q1["Z"]*q2["Z"]
    q["X"]=q1["W"]*q2["X"]+q1["X"]*q2["W"]+q1["Y"]*q2["Z"]-q1["Z"]*q2["Y"]
    q["Y"]=q1["W"]*q2["Y"]-q1["X"]*q2["Z"]+q1["Y"]*q2["W"]+q1["Z"]*q2["X"]
    q["Z"]=q1["W"]*q2["Z"]+q1["X"]*q2["Y"]-q1["Y"]*q2["X"]+q1["Z"]*q2["W"]
    return q
    
def getBearing():
    """Returns the XY Orientation as a bearing unit vector"""
    return bearing(getPose()['Pose']['Orientation'])

def pos2coor(position,gridsmap):
    """Convert Position to coordinate on map grid"""
    coordinates = [int(floor((gridsmap.upper_right_y - position[1]) * 2)), int(floor((position[0] - gridsmap.lower_left_x) * 2))]
    return coordinates

def coor2pos(coordinates,gridmap):
    '''convert coordinates to X,Y position. Assume point is the centre of the coordinate grid'''
    position = [float(coordinates[1] - abs(gridmap.lower_left_x) * 2) / 2 , float(gridmap.upper_right_y * 2 - coordinates[0]) / 2]
    return position

class robot():
    def __init__(self):
        self.position = [] # X,Y position
        self.coordinate = [] # Row Column coordinate in grid map
        self.speed = [] # Angular and Linear speed
        self.orientation = [] # Robot orientation angle in original frame in degrees
        self.target = [] # coordinate of target point
        self.scan_data = [] # From right most to left most scan boundary points position 
    
    def get_position(self):
        pos = getPose()
        self.position = [pos['Pose']['Position']['X'], pos['Pose']['Position']['Y']]
        return self.position
    
    def get_coor(self, grids):
        self.coordinate = pos2coor(self.position, grids)
        return self.coordinate
    
    def set_speed(self,a,l):
        postSpeed(a, l)
        self.speed = [a,l]
        return self.speed
    
    def get_orientation(self):
        orien_vector = getBearing()
        self.orientation = degrees(atan2(orien_vector['Y'],orien_vector['X']))
        return self.orientation
    
    def set_target(self,r,c):
        self.target = [r,c]
    
    def scan(self, grids):
        '''
        # Adjust direction before scan
        direction = self.get_orientation()
        angle_bias = 'infinity'
        if direction not in range(-10,11) or range(-100,-79) or range(80,101) or range(170,181) or range(-180, 169):
            for angles in [direction, direction - 90, direction + 90]:
                if abs(angles) < angle_bias:
                    angle_bias = angles
            self.set_speed(angle_bias, 0)
            time.sleep(abs(radians(angle_bias)))
        self.set_speed(0, 0)
        '''
        
        distance = getLaser()['Echoes']
        p = self.position
        right_avg = 0
        for i in range(40, 51):
            right_avg += distance[i] / 10
        forward_avg = 0
        for i in range(130, 141):
            forward_avg += distance[i] / 10
        left_avg = 0
        for i in range(220, 231):
            left_avg += distance[i] / 10
        
        # Check obstacles
        if right_avg < 4:
            self.right_wall = True
        else:
            self.right_wall = False
        if left_avg < 4:
            self.left_wall = True
        else:
            self.left_wall = False
        if forward_avg < 4 or min([distance[i] for i in range(132,139)]) < 4 :
            self.forward_wall = True
        else:
            self.forward_wall = False 
            
        #Check boundary
        if p[0] - grids.lower_left_x < 8: # near left boundary
            if self.orientation < -90 or self.orientation > 90:
                self.forward_wall = True
            elif self.orientation > -135 and self.orientation < -45:
                self.right_wall = True
            elif self.orientation > 45 and self.orientation < 135:
                self.left_wall = True
        elif grids.upper_right_x - p[0]< 8: # near right boundary
            if self.orientation > -90 and self.orientation < 90:
                self.forward_wall = True
            elif self.orientation > -135 and self.orientation < -45:
                self.left_wall = True
            elif self.orientation > 45 and self.orientation < 135:
                self.right_wall = True
        
        if p[1] - grids.lower_left_y < 8: # near lower boundary
            if self.orientation > -180 and self.orientation < 0:
                self.forward_wall = True
            elif self.orientation > -45 and self.orientation < 45:
                self.right_wall = True
            elif self.orientation < -135 or self.orientation > 135:
                self.left_wall = True
        elif grids.upper_right_y - p[1] < 8: # near upper boundary
            if self.orientation > 0 and self.orientation < 180:
                self.forward_wall = True
            elif self.orientation > -45 and self.orientation < 45:
                self.left_wall = True
            elif self.orientation < -135 or self.orientation > 135:
                self.right_wall = True
        
        #print 'Front :', self.forward_wall, forward_avg, 'Right :',self.right_wall, right_avg ,'Left :', self.left_wall, left_avg, 'Position: ', p,self.orientation
        beta = self.get_orientation() - 135 
        self.scan_data = []
        for i in range(271):
            point = [distance[i] * cos(radians(beta)) + p[0] , distance[i] * sin(radians(beta)) + p[1]]
            beta += 1
            self.scan_data.append(point)
        return self.scan_data
    
    def update_map(self,grids):
        scan_range = []
        for point in self.scan_data:
            if point[0] <= grids.lower_left_x:
                point[0] = grids.lower_left_x
            elif point[0] >= grids.upper_right_x:
                point[0] = grids.upper_right_x - 0.1
            if point[1] <= grids.lower_left_y:
                point[1] = grids.lower_left_y + 0.1
            elif point[1] >= grids.upper_right_y:
                point[1] = grids.upper_right_y
            scan_range.append(pos2coor(point, grids))
        scan_range.append(self.coordinate)
        grids.update(scan_range)
        self.scan_boundary = scan_range
        return scan_range
    
    def scan_and_update(self,grids):
        ''' 
        Stop then scan and update 
        '''
        self.set_speed(0, 0)
        self.get_position()
        self.get_coor(grids)
        self.get_orientation()
        self.scan(grids)
        self.update_map(grids)
        
    def find_target(self,grids):
        '''
        Find a target point in a never been block to move, return the coordinate of that point
        '''
        choices = []
        for point in self.scan_boundary:
            point_block = (point[0] // 2, point[1] // 2)
            print point,point_block
            if grids.check_unkown(point_block):
                if point_block not in choices:
                    if not grids.block_occupancy(point_block):
                        choices.append(point_block)
        choices.sort()
        
        self.target = random.choice(choices)
        return self.target
    
    def search_path(self,grids):
        '''
        Wavefront path planning 
        Generate a path from current position to target position
        plan path on a larger scale block map
        '''
        current_block = (self.coordinate[0] // 2, self.coordinate[1] // 2)
        self.path = grids.search_path(current_block, self.target)
        return self.path
    
    def follow_path(self, grids):
        '''
        Follow the path from current point to target point
        '''
        for i in range(1, len(self.path)):
            self.scan_and_update(grids)
            if grids.block_occupancy(self.path[i]):
                break
            block_centre = coor2pos(grids.block_center(self.path[i]) , grids)
            angle = atan2(block_centre[1] - self.position[1] , block_centre[0] - self.position[0])
            #print 'angle :', degrees(angle)
            distance = sqrt((block_centre[0] - self.position[0])**2 + (block_centre[1] - self.position[1])**2 )
            print 'Current Position:', self.position, 'Heading to: ', self.path[i],' at ', block_centre
            if angle > radians(self.orientation):
                t = angle - radians(self.orientation)
                self.set_speed(0.5, 0)
                time.sleep(t * 2)
            else:
                self.set_speed(-0.5, 0)
                t = radians(self.orientation) - angle
                time.sleep(t * 2)
            self.set_speed(0, 0)
            d = getLaser()['Echoes']
            b = False
            for i in range(130,141):
                if d[i] < 4:
                    b = True
            if b == True:
                break    
            self.set_speed(0, 1)
            time.sleep(distance)
            self.set_speed(0, 0)
            grids.show()
        self.set_speed(0, 0)   
        
    def go(self,gridsmap):
        self.scan_and_update(gridsmap)
        self.find_target(gridsmap)
        self.search_path(gridsmap)
        self.follow_path(gridsmap)
        
        
class gridmap():
    def __init__(self,a,b,c,d):
        self.lower_left_x = a
        self.lower_left_y = b
        self.upper_right_x = c
        self.upper_right_y = d
        self.width = -a + c
        self.height = -b + d
        self.boundary = []
        # HIMM 0
        #Set 6 means the initial state is unknown
        self.grid = [[6 for col in range((self.width) * 2)]
                        for row in range((self.height) * 2)]
        self.known_area = [[0 for col in range((self.width) * 2)]
                               for row in range((self.height) * 2)]
        #separate map into several 1mX1m blocks for setting target 
        #initial value 0 indicate that block the robot was never been 
        self.blocks = [[0 for i in range(self.width / 1)] for j in range(self.height / 1)]
        self.block_map = [[0 for i in range(self.width / 1)] for j in range(self.height / 1)]
        self.block_idx = []
        for row in range(len(self.blocks)):
            for col in range(len(self.blocks[0])):
                self.block_idx.append((row,col))
        '''
        #draw border
        for i in range(self.height * 2):
            self.grid[i][0] = 15
            self.grid[i][-1] = 15
        for i in range(self.width * 2):
            self.grid[0][i] = 15
            self.grid[-1][i] = 15
         '''   
                    
    def show(self): 
        #for i in range(len(self.grid)):
        #    print self.grid[i]
        plt.imshow(self.grid)
        plt.savefig('MAP.jpg')
        
    def reset_scan_area(self):
        self.known_area = [[1 for col in range((self.width) * 2)]
                               for row in range((self.height) * 2)] 
        
    def update(self,area):
        self.boundary = area
        r0 = area[-1][0]; c0 = area[-1][1]
        self.blocks[r0 // 2][c0 // 2] = 1 #update block status, current block has been explored
        for i in range(271):
            r1 = area[i][0]; c1 = area[i][1]
            self.known_area[r1][c1] = 1
            if self.grid[r1][c1] <= 12: # HIMM 
                self.grid[r1][c1] += 3
            delta_r = r1 - r0
            if delta_r == 0:
                delta_r = 1
            delta_c = c1 - c0
            if delta_c == 0:
                delta_c = 1
            if abs(delta_c) >= abs(delta_r):
                if delta_c > 0:
                    step_c = 1
                else:
                    step_c = -1
                step_r = float(delta_r) / abs(delta_c)
                for j in range(abs(delta_c)):
                    if self.grid[min(r0 + int(round(j * step_r)), self.height * 2 - 1)][min(c0 + j * step_c, self.height * 2 - 1)] > 0:
                        self.grid[min(r0 + int(round(j * step_r)), self.height * 2 - 1)][min(c0 + j * step_c, self.height * 2 -1)] -= 1 # HIMM 1
                        self.known_area[min(r0 + int(round(j * step_r)), self.height * 2 - 1)][min(c0 + j * step_c, self.height * 2 -1)] = 1
            elif abs(delta_c) < abs(delta_r):
                step_c = float(delta_c) / abs(delta_r)
                if delta_r > 0:
                    step_r = 1
                else:
                    step_r = -1
                for j in range(abs(delta_r)):
                    if self.grid[min(r0 + j * step_r, self.width * 2 - 1)][min(c0 + int(round(j * step_c)), self.width * 2 - 1)] > 0:
                        self.grid[min(r0 + j * step_r, self.width * 2 - 1)][min(c0 + int(round(j * step_c)), self.width * 2 - 1)] -= 1 # HIMM 1
                        self.known_area[min(r0 + j * step_r, self.width * 2 - 1)][min(c0 + int(round(j * step_c)), self.width * 2 - 1)] = 1
            
    def check_unkown(self,block_coord):
        if self.blocks[block_coord[0]][block_coord[1]] == 0:
            return True
        else:
            return False
        
    def block_center(self,block_coord):
        return (block_coord[0] * 2 + 2 , block_coord[1] * 2 + 2)
    
    def block_occupancy(self,block_coord):
        row_s = block_coord[0] * 2
        row_e = row_s + 2
        col_s = block_coord[1] * 2
        col_e = col_s + 2
        for r in range(row_s, row_e):
            for c in range(col_s, col_e):
                if self.grid[r][c] >= 12:
                    return True       
        return False    
    
    def reset_blockmap(self):
        self.block_map = [[0 for i in range(self.width / 1)] for j in range(self.height / 1)]
    
    def search_path(self,start_block,target_block):
        '''
        use wavefront algorithm to find a path 
        '''
        self.reset_blockmap()
        print 'Start block:',start_block,' Target Block:',target_block
        #wavefront : update map value to navigate
        waves = [target_block]
        n = 2
        self.block_map[target_block[0]][target_block[1]] = 2
        while start_block not in waves:
            n += 1
            temp_wave = set()
            for block in waves:
                #print block
                block_w = (block[0], max(block[1] - 1 , 0))              
                block_e = (block[0], min(block[1] + 1, len(self.block_map[0]) - 1))
                block_n = (max(block[0] - 1 , 0), block[1])
                block_s = (min(block[0] + 1,len(self.block_map) - 1), block[1])
                if start_block in [block_e,block_n,block_s,block_w]:
                    temp_wave.add(start_block)
                if not self.block_occupancy(block_w):
                    if self.block_map[block_w[0]][block_w[1]] == 0:
                        temp_wave.add(block_w)
                if not self.block_occupancy(block_e):
                    if self.block_map[block_e[0]][block_e[1]] == 0:
                        temp_wave.add(block_e)
                if not self.block_occupancy(block_n):
                    if self.block_map[block_n[0]][block_n[1]] == 0:
                        temp_wave.add(block_n)
                if not self.block_occupancy(block_s):
                    if self.block_map[block_s[0]][block_s[1]] == 0:
                        temp_wave.add(block_s)
            for move in temp_wave:
                self.block_map[move[0]][move[1]] = n
            waves = list(temp_wave) 
            #for i in range(len(self.block_map)):
            #    print i,self.block_map[i]
            #print ''      
        #find one way
        path = []
        value = self.block_map[start_block[0]][start_block[1]]
        path.append(start_block)
        while target_block not in path:
            value -= 1
            move = path[-1]
            move_n = (max(move[0] - 1 , 0), move[1])
            move_s = (min(move[0] + 1,len(self.block_map) - 1), move[1])
            move_e = (move[0], min(move[1] + 1, len(self.block_map[0]) - 1))
            move_w = (move[0], max(move[1] - 1 , 0))
            if self.block_map[move_n[0]][move_n[1]] == value:
                path.append(move_n)
            elif self.block_map[move_e[0]][move_e[1]] == value:
                path.append(move_e)
            elif self.block_map[move_s[0]][move_s[1]] == value:
                path.append(move_s)
            elif self.block_map[move_w[0]][move_w[1]] == value:
                path.append(move_w)
        print path 
              
        return path
    
if __name__ == '__main__':
    global MRDS_URL
    
    input_para = []
    for i in range(len(sys.argv)-1):
        input_para.append(sys.argv[i+1])

    print input_para
    MRDS_URL = input_para[0]
    if 'http://' in MRDS_URL:
        MRDS_URL.replace('http://','')
    a = int(input_para[1])
    b = int(input_para[2])
    c = int(input_para[3])
    d = int(input_para[4])


    newGrid = gridmap(a,b,c,d)
    print len(newGrid.blocks),len(newGrid.blocks[0])
    #newGrid.show()
    
    print 'Sending commands to MRDS server', MRDS_URL
    
    newROBO = robot()

    
    t1 = time.time()
    while (1):
        newROBO.go(newGrid)
        t2 = time.time()
        if t2 - t1 > 5:
            t1 = t2
            newGrid.show()

    
    newGrid.show()
    
    
   
    
    
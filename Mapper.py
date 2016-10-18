"""
Example demonstrating how to communicate with Microsoft Robotic Developer
Studio 4 via the Lokarria http interface. 

Author: Erik Billing (billing@cs.umu.se)

Updated by Ola Ringdahl 204-09-11
"""


import httplib, json, time, sys
from math import sin,cos,pi,atan2,degrees,radians, sqrt
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
    coordinates = [int(round((gridsmap.upper_right_y - position[1]) * 2)), int(round((position[0] - gridsmap.lower_left_x) * 2))]
    return coordinates

def coor2pos(coordinates,gridmap):
    '''convert coordinates to X,Y position. Assume point is the centre of the coordinate grid'''
    position = [float(coordinates[1] - abs(gridmap.lower_left_x) * 2) / 2 , float(gridmap.upper_right_y * 2 - coordinates[0]) / 2]
    return position

class robot():
    def __init__(self):
        self.position = []
        self.coordinate = []
        self.speed = []
        self.orientation = []
        self.target = []
        self.scan_data = []
    
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
        
        print ('Front :', self.forward_wall, forward_avg, 'Right :',self.right_wall, right_avg ,'Left :', self.left_wall, left_avg, 'Position: ', p,self.orientation)
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
                point[0] = grids.upper_right_x
            if point[1] <= grids.lower_left_y:
                point[1] = grids.lower_left_y
            elif point[1] >= grids.upper_right_y:
                point[1] = grids.upper_right_y
            scan_range.append(pos2coor(point, grids))
        scan_range.append(self.coordinate)
        grids.update(scan_range)
        self.scan_boundary = scan_range
        return scan_range
    
    def scan_and_update(self,grids):
        ''' Stop then scan and update '''
        self.set_speed(0, 0)
        self.get_position()
        self.get_coor(grids)
        self.get_orientation()
        self.scan(grids)
        self.update_map(grids)
        
    def plan_and_move(self, grids):
        #time.sleep(1)
        distance = getLaser()['Echoes']
        direction = self.orientation
        if self.forward_wall == False:
            print 'Go forward'
            self.set_speed(0, 1) # Go straight 1m
            time.sleep(1)
            self.set_speed(0, 0)
        else:
            left_count = 0
            right_count = 0
            left_end = 0
            right_end = 0
            for i in range(10, 100):
                p_left = self.scan_boundary[135 + i]
                p_right = self.scan_boundary[135 - i]
                if sqrt((p_left[0] - self.scan_boundary[-1][0]) ** 2 + (p_left[1] - self.scan_boundary[-1][1]) ** 2) > 14:
                    left_count += 1
                    left_end = i
                else:
                    if left_count < 30:
                        left_count = 0
                if sqrt((p_right[0] - self.scan_boundary[-1][0]) ** 2 + (p_right[1] - self.scan_boundary[-1][1]) ** 2) > 14:
                    right_count += 1
                    right_end = i
                else:
                    if right_count < 30:
                        right_count = 0
                #if left_count > 29 or right_count > 29:
                    #break
            if left_count < 29 and right_count < 29:
                print ('U - Turn', ' Left count: ',left_count,' Right count: ',right_count)
                self.set_speed(1, 0)
                time.sleep(pi)
                self.set_speed(0, 0)
            else:
                if left_count > right_count:
                    #turn left and go 1m
                    print ('Turn left ',left_count,'  ',p_left)
                    self.set_speed(0.2, 0)
                    time.sleep(radians(left_end - left_count + left_count / 2) * 5)
                    self.set_speed(0, 1)
                    time.sleep(1)
                    self.set_speed(0, 0)
                elif right_count > left_count:
                    # turn right and go 1m
                    print ('Turn right ',right_count,'  ',p_right)
                    self.set_speed(-0.2, 0)
                    time.sleep(radians(right_end - right_count + right_count / 2) * 5)
                    self.set_speed(0, 1)
                    time.sleep(1)
                    self.set_speed(0, 0)
                else:
                    if self.left_wall == True:
                        # turn right
                        print ('Turn right ',right_count,'  ',p_right)
                        self.set_speed(-0.2, 0)
                        time.sleep(radians(right_end - right_count + right_count / 2) * 5)
                        self.set_speed(0, 1)
                        time.sleep(1)
                        self.set_speed(0, 0)
                    elif self.right_wall == True:
                        # turn left
                        print ('Turn left ',left_count,'  ',p_left)
                        self.set_speed(0.2, 0)
                        time.sleep(radians(left_end - left_count + left_count / 2) * 5)
                        self.set_speed(0, 1)
                        time.sleep(1)
                        self.set_speed(0, 0)
                    else:
                        left_distance = [sqrt((self.scan_boundary[i][0] - self.scan_boundary[-1][0]) ** 2 + (self.scan_boundary[i][1] - self.scan_boundary[-1][1]) ** 2) for i in range(220,231)]
                        right_distance = [sqrt((self.scan_boundary[i][0] - self.scan_boundary[-1][0]) ** 2 + (self.scan_boundary[i][1] - self.scan_boundary[-1][1]) ** 2) for i in range(40,51)]
                        if sum(left_distance) > sum(right_distance):
                            #turn left
                            print ('Turn left ',left_count,'  ',p_left)
                            self.set_speed(0.2, 0)
                            time.sleep(radians(left_end - left_count + left_count / 2) * 5)
                            self.set_speed(0, 1)
                            time.sleep(1)
                            self.set_speed(0, 0)
                        elif sum(left_distance) < sum(right_distance):
                            #turn right
                            print ('Turn right ',right_count,'  ',p_right)
                            self.set_speed(-0.2, 0)
                            time.sleep(radians(right_end - right_count + right_count / 2) * 5)
                            self.set_speed(0, 1)
                            time.sleep(1)
                            self.set_speed(0, 0)
         
    def go(self,gridsmap):
        self.scan_and_update(gridsmap)
        self.plan_and_move(gridsmap)
        
        
        
        
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
        self.grid = [[100 for col in range((self.upper_right_x - self.lower_left_x) * 2 + 1)]
                          for row in range((self.upper_right_y - self.lower_left_y) * 2 + 1)]
        self.scan_area = [[1 for col in range((self.upper_right_x - self.lower_left_x) * 2 + 1)]
                               for row in range((self.upper_right_y - self.lower_left_y) * 2 + 1)]
        
        #draw border
        for i in range(self.height * 2):
            self.grid[i][0] = 15
            self.grid[i][-1] = 15
        for i in range(self.width * 2):
            self.grid[0][i] = 15
            self.grid[-1][i] = 15
            
                    
    def show(self):
        #for i in range(len(self.grid)):
            #print self.grid[i]
        plt.imshow(self.grid)
        plt.savefig('MAP.jpg')
        
    def reset_scan_area(self):
        self.scan_area = [[1 for col in range((self.upper_right_x - self.lower_left_x) * 2 + 1)]
                               for row in range((self.upper_right_y - self.lower_left_y) * 2 + 1)] 
        
    def update(self,area):
        self.reset_scan_area()
        self.boundary = area
        r0 = area[-1][0]; c0 = area[-1][1]
        for i in range(271):
            r1 = area[i][0]; c1 = area[i][1]
            self.scan_area[r1][c1] = 1
            if self.grid[r1][c1] < 100: # HIMM 15
                self.grid[r1][c1] += 10 # HIMM +3
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
                        self.grid[min(r0 + int(round(j * step_r)), self.height * 2 - 1)][min(c0 + j * step_c, self.height * 2 -1)] -= 5 # HIMM -1
                        self.scan_area[min(r0 + int(round(j * step_r)), self.height * 2 - 1)][min(c0 + j * step_c, self.height * 2 -1)] = 0
            elif abs(delta_c) < abs(delta_r):
                step_c = float(delta_c) / abs(delta_r)
                if delta_r > 0:
                    step_r = 1
                else:
                    step_r = -1
                for j in range(abs(delta_r)):
                    if self.grid[min(r0 + j * step_r,self.width * 2 - 1)][min(c0 + int(round(j * step_c)), self.width * 2 - 1)] > 0:
                        self.grid[min(r0 + j * step_r, self.width * 2 - 1)][min(c0 + int(round(j * step_c)), self.width * 2 - 1)] -= 5 # HIMM -1
                        self.scan_area[min(r0 + j * step_r, self.width * 2 - 1)][min(c0 + int(round(j * step_c)), self.width * 2 - 1)] = 0
    

     
if __name__ == '__main__':
    '''
    input_para = []
    for i in range(len(sys.argv)-1):
        input_para.append(sys.argv[i+1])
    
    MRDS_URL = input_para[1]
    a = input_para[2]
    b = input_para[3]
    c = input_para[4]
    d = input_para[5]
    '''
    
    MRDS_URL = '192.168.1.88:50000'
    a = -30; b = -20; c = 40; d = 45

    newGrid = gridmap(a,b,c,d)
    #newGrid.show()
    
    print ('Sending commands to MRDS server', MRDS_URL)
    
    newROBO = robot()

    t1 = time.time()
    while (1):
        newROBO.go(newGrid)
        t2 = time.time()
        if t2 - t1 > 5:
            t1 = t2
            newGrid.show()
    
    newGrid.show()
    
    
   
    
    
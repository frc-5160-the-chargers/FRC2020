#FILE IS DOMAIN OF HARRISON
#DO NOT TOUCH SEMICOLONS
import math
from robotmap import RobotMap
from components.sensors import Encoders,EncoderSide, NavX
from networktables import NetworkTables
#from wpilib import SmartDashboard


class PosApprox:
    SUPER_SIMPLE = -1;
    ENCODER_ONLY = 0; #just here as a control lmao
    LEFT_ENCODER_AND_GYRO = 1;
    RIGHT_ENCODER_AND_GYRO = 2;
    OUTER_ENCODER_AND_GYRO = 3
    ENCODER_RC_GYRO_ARC = 4;
    AVERAGE_ENCODER_AND_GYRO = 5;

    encoders: Encoders;
    navx: NavX;

    ##TODO: put somewhere that makes sense lol
    

    def __init__(self):
        self.frame = None;
        location_settings = [[PosApprox.ENCODER_ONLY],
            [PosApprox.LEFT_ENCODER_AND_GYRO],
            [PosApprox.RIGHT_ENCODER_AND_GYRO],
            [PosApprox.LEFT_ENCODER_AND_GYRO,PosApprox.RIGHT_ENCODER_AND_GYRO],
            [PosApprox.OUTER_ENCODER_AND_GYRO],
            [PosApprox.ENCODER_RC_GYRO_ARC],
            [PosApprox.AVERAGE_ENCODER_AND_GYRO]]; 
        self.setting = PosApprox.ENCODER_RC_GYRO_ARC;
        self.location = Location(self.setting);
        self.last_positions =[0,0,0]; #left encoder, right encoder, gyro

    def execute(self):
        inputStates = [self.encoders.get_position(EncoderSide.LEFT),self.encoders.get_position(EncoderSide.RIGHT),self.navx.get_heading()];
        #print(inputStates);
        sd = NetworkTables.getTable('SmartDashboard');
        sd.putNumberArray('locations/states',inputStates)
        deltas = [inputStates[i] - self.last_positions[i] for i in range(3)];
        self.update(*deltas);
        self.last_positions = inputStates;

    def get_location(self):
        return self.location;

    def reset(self):
        self.last_positions = [0,0,0];

    def reset_location(self): #unsure whether to call from reset
        self.location.reset();

    def update(self,dL,dR,dTheta):
        print("updated");
        sd = NetworkTables.getTable('SmartDashboard');
        sd.putNumber('locations/num_locations',len(self.locations));
        sd.putNumber('locations/dL',dL);
        sd.putNumber('locations/dR',dR);
        sd.putNumber('locations/dTheta',dTheta);
        self.location.updateLocation([PosApprox.get_location_offsets(dL,dR,dTheta,type) for type in self.location.types]); #multiple types means average their outputs together
        #print(location);
        if self.names is not None:
            sd.putStringArray('locations/location_names',self.names);

        
        
        
    @classmethod
    def get_location_offsets(cls,dL,dR,dTheta,approxType): #NOTE: dP is (forward,horizontal) NOT (x,y); make sure to orient to robot forward when updating.
        WHEEL_DISTANCE = RobotMap.Drivetrain.distance_between_wheels;
        #print("thing");
        dL = dL*RobotMap.Encoders.distance_per_pulse; #delta left encoder
        dR = dL*RobotMap.Encoders.distance_per_pulse; #delta right encoder
        if (dL == dR or (dTheta == 0 and approxType != cls.ENCODER_ONLY)):
            out = ((0,(dL + dR)/2),0);
            return out;

        print(f"dL: {dL}");
        print(f"dR: {dR}");
        print(f"dTheta: {dTheta}");

        leftOutside = abs(dL) > abs(dR);
        outsideCoeff = 1 if leftOutside else -1;

        rC = None;
        if (approxType == cls.SUPER_SIMPLE):
            return (((dL + dR)/2,0),dTheta);

        elif (approxType == cls.ENCODER_ONLY):
            #rC: central radius - radius of the circle halfway between wheels
            rC = WHEEL_DISTANCE/2*(dR + dL) / (dL - dR) *  outsideCoeff;
            print("encoder RC: ", rC);

            #known arclength, R, over known radius of R, which is shifted from center by a certain amount
            if (dL == 0):
                dTheta = dR/(rC -  outsideCoeff*WHEEL_DISTANCE/2);
            else:
                dTheta = dL/(rC +  outsideCoeff*WHEEL_DISTANCE/2);
        elif (approxType == cls.OUTER_ENCODER_AND_GYRO):
            outsideArc = dL if leftOutside else dR;
            rC = -outsideCoeff*WHEEL_DISTANCE/2 + outsideCoeff*outsideArc/dTheta;

        elif (approxType == cls.LEFT_ENCODER_AND_GYRO):
            rC = -outsideCoeff*WHEEL_DISTANCE/2 + dL/dTheta;

        elif (approxType == cls.RIGHT_ENCODER_AND_GYRO):
            rC = -outsideCoeff*WHEEL_DISTANCE/2 + dR/dTheta;

        elif (approxType == cls.ENCODER_RC_GYRO_ARC):
            rC = WHEEL_DISTANCE/2*(dR + dL) / (dL - dR) * outsideCoeff;

        elif (approxType == cls.AVERAGE_ENCODER_AND_GYRO):
            rC = outsideCoeff*(dR + dL)/(2 * dTheta);


        dP = (rC * (1-math.cos(dTheta)),rC * math.sin(dTheta)); #change in position

        out = (dP,-1*dTheta*(1 if leftOutside else -1));
        #print(out);
        return out;

class Location:
    def __init__(self,approxTypes,startPos=[0,0],startAngle=0):
        self.pos = startPos;
        self.startPos = startPos.copy();
        self.angle = startAngle;
        self.startAngle = startAngle;
        self.types = approxTypes;

    def reset(self,startPos=self.startPos.copy(),startAngle=self.startAngle):
        self.pos = startPos;
        self.angle = startAngle;

    def __str__(self):
        return f"robot location: Pos: {self.pos}, Angle:{self.angle}, Approximation Types: {self.types}";

    def updateLocation(self,offsets):
        cumulativeDP = [0,0];
        cumulativeDTheta = 0;
        #print(offsets);
        for offset in offsets:
            dP = offset[0];
            dTheta = offset[1];
            cumulativeDP[0] += dP[0] * math.cos(self.angle) - dP[1] * math.sin(self.angle);
            cumulativeDP[1] += dP[0] * math.sin(self.angle) + dP[1] * math.cos(self.angle);
            cumulativeDTheta += dTheta; 
        cumulativeDP[0] /= len(offsets);
        cumulativeDP[1] /= len(offsets);
        dTheta /= len(offsets);

        self.pos[0] += cumulativeDP[0]; self.pos[1] += cumulativeDP[1]; self.angle += cumulativeDTheta;

    def toArray(self):
        return [self.pos[0],self.pos[1],self.angle];

        

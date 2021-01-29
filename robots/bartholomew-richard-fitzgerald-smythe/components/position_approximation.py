#FILE IS DOMAIN OF HARRISON
#DO NOT TOUCH SEMICOLONS
import math
from sensors import Encoders,EncoderSide, NavX
from networktables import NetworkTables


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
    encoder_scale = 5; #encoder input units -> real world units
    WHEEL_DISTANCE = 20; #distance between wheels

    def __init__(self,location_settings,names=None):
        self.locations = [Location(setting) for setting in location_settings]; #TODO: Clean Up
        self.names = names;
        self.last_positions =[0,0,0]; #left encoder, right encoder, gyro

    def execute(self):
        inputStates = [self.encoders.get_position(EncoderSide.LEFT),self.encoders.get_position(EncoderSide.RIGHT),self.navx.get_heading()];
        deltas = [inputStates[i] - self.last_positions[i] for i in range(3)];
        self.update(*deltas);
        self.last_positions = inputStates;
        


    def reset(self):
        self.last_positions = [0,0,0];


    def update(self,dL,dR,dTheta):
        for location in self.locations:
            location.updateLocation([PosApprox.get_location_offsets(dL,dR,dTheta,type) for type in location.types]); #multiple types means average their outputs together
        sd = NetworkTables.getTable('SmartDashboard');
        sd.putNumberArray('locations/locations',[location.toArray() for location in self.locations]);
        if self.names is not None:
            sd.putStringArray('locations/location_names',self.names);
        
        

    @classmethod
    def get_location_offsets(cls,dL,dR,dTheta,approxType): #NOTE: dP is (forward,horizontal) NOT (x,y); make sure to orient to robot forward when updating.
        dL = dL*cls.encoder_scale; #delta left encoder
        dR = dL*cls.encoder_scale; #delta right encoder
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
            rC = cls.WHEEL_DISTANCE/2*(dR + dL) / (dL - dR) *  outsideCoeff;
            print("encoder RC: ", rC);

            #known arclength, R, over known radius of R, which is shifted from center by a certain amount
            if (dL == 0):
                dTheta = dR/(rC -  outsideCoeff*cls.WHEEL_DISTANCE/2);
            else:
                dTheta = dL/(rC +  outsideCoeff*cls.WHEEL_DISTANCE/2);
        elif (approxType == cls.OUTER_ENCODER_AND_GYRO):
            outsideArc = dL if leftOutside else dR;
            rC = -outsideCoeff*cls.WHEEL_DISTANCE/2 + outsideCoeff*outsideArc/dTheta;

        elif (approxType == cls.LEFT_ENCODER_AND_GYRO):
            rC = -outsideCoeff*cls.WHEEL_DISTANCE/2 + dL/dTheta;

        elif (approxType == cls.RIGHT_ENCODER_AND_GYRO):
            rC = -outsideCoeff*cls.WHEEL_DISTANCE/2 + dR/dTheta;

        elif (approxType == cls.ENCODER_RC_GYRO_ARC):
            rC = cls.WHEEL_DISTANCE/2*(dR + dL) / (dL - dR) * outsideCoeff;

        elif (approxType == cls.AVERAGE_ENCODER_AND_GYRO):
            rC = outsideCoeff*(dR + dL)/(2 * dTheta);


        dP = (rC * (1-math.cos(dTheta)),rC * math.sin(dTheta)); #change in position

        out = (dP,-1*dTheta*(1 if leftOutside else -1));
        return out;

class Location:
    def __init__(self,approxTypes,startPos=[0,0],startAngle=0):
        self.pos = startPos;
        self.angle = startAngle;
        self.types = approxTypes;

    def updateLocation(self,offsets):
        cumulativeDP = [0,0];
        cumulativeDTheta = 0;
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

        

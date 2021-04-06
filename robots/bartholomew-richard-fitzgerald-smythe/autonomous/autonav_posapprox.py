from magicbot import AutonomousStateMachine, state

from components.drivetrain import Drivetrain
from components.sensors import EncoderSide
from components.position_approximation import PosApprox

from robotmap import RobotMap

import math

class Path:
    def __init__(self,power,nodes):
        self.power = power;
        self.nodes = nodes;

    def __len__(self):
        return len(self.nodes);
    
    def __getitem__(self,key):
        if key >= 0:
            return self.nodes[key];
        else:
            return [0,0];

class Paths:
    SLALOM = [Path(-RobotMap.Drivetrain.max_auto_power,[18.885447407122943, 14.420321500506644], [5.3630220011044045, 10.490048460822837], [-15.57279580022347, 7.994760358956118], [-41.030898791659865, 7.178945067641089], [-61.905282858312766, 7.616283152275643], [-74.8211148206705, 8.459807505693211], [-81.82300282437213, 9.143122999137242], [-84.8627123692807, 9.413165401999601], [-84.73442330138798, 9.178958001502094], [-81.15247218044782, 8.445268306863824], [-72.43357074225685, 7.330877501489462], [-55.603689730129986, 6.243262566334247], [-34.62270862745415, 5.912176443513411], [-16.749027479776384, 6.341812884048806], [-4.460361554262448, 7.1708709791534435], [3.1863329512715346, 8.106880348719828], [7.654549538918863, 8.988447188293893], [9.948462568638073, 9.73758735075557], [10.641002620320602, 10.320268215654446], [10.025638336338792, 10.727035019246573], [8.219310126573633, 10.965113311357012], [5.221044473412943, 11.056412294787258], [0.950798541711964, 11.038926412376334], [-3.970925833754791, 10.9570688726896], [-7.57148390202987, 10.778085304880587], [-9.545322016958497, 10.440879878626367], [-9.830529463253141, 9.912943677931908], [-8.153675374130536, 9.190223910704342], [-3.8956094182353858, 8.305300988490474], [4.095187733820016, 7.3525336120645335], [17.474817315012196, 6.542199447276952], [36.86922007640652, 6.256994700815104], [57.72742125576115, 6.892940382159211], [70.44702495449451, 8.080816366549572], [75.36208122289028, 9.116237437971755], [75.28267075022522, 9.7625705697818], [71.74131965982359, 9.997940135526251], [66.58615236695817, 9.83563647729956], [59.54280536248135, 9.334479022509356], [49.488355028064525, 8.623839441265877], [34.94568374956332, 7.953930685970602], [15.290438949258673, 7.758688923777395], [-4.925037005441426, 8.347176782444109], [-20.06806792697074, 9.16512337865991], [-31.840840631169655, 9.775181073662623], [-42.37137182369608, 10.020267587866318], [-53.337827330615205, 9.889220736033932], [-66.34192047965895, 9.494492416216827], [-82.94812979756401, 9.113047531495887], [-101.81357415045542, 9.171975216908816], [-117.41337260177117, 9.578628889051448], [-129.8119444610308, 9.921253173613884], [-140.5857912275553, 9.972343418329421], [-151.3013038561725, 9.652106483747062], [-163.6698961883086, 9.001476717469842], [-179.86269850019426, 8.220255371428], [-200.25990467412234, 7.823523167244701], [-218.45627259672372, 8.156821720189942], [-231.99339457966724, 8.833189354296604], [-241.7681643420308, 9.500791610439158], [-249.0999611783078, 9.959875277393419], [-254.99541286569777, 10.106572205198278], [-259.3295541517285, 9.921545090665356], [-260.67190605609886, 9.40741340734818], [-258.39409865029785, 8.537380679903292], [-250.93730968408101, 7.394963030009189], [-235.07802923781955, 6.307172094432136], [-212.76097009756793, 5.9801966063384695], [-194.24578723267499, 6.517601611293963], [-182.32381830391722, 7.458181498427017], [-175.46936856228348, 8.460002172348542], [-171.9604104383362, 9.353665797292225], [-170.75540191645857, 10.067311554893637], [-171.32483319385554, 10.579050678094394], [-173.4490132576415, 10.897287404332747], [-177.0961577063856, 11.055097319007537], [-181.7871628209939, 11.112233862376957], [-185.6372459411737, 11.093820298395254], [-188.30190882845585, 10.959423320849197], [-189.83943856685195, 10.677117898898612], [-190.2274173523831, 10.22915209916407], [-189.32519897789356, 9.611263687559754], [-186.81230431894878, 8.835323374712177], [-182.0802624764776, 7.937950115087316], [-174.07857511514214, 7.000917711028856], [-161.31935972204062, 6.191665313385935], [-143.00613118494724, 5.807522973544559], [-122.02644851086102, 6.19535387618696], [-105.92231978944305, 7.275179960300496], [-97.34375405856085, 8.379076448345659], [-93.36167559369312, 9.174050980153647], [-92.38058059456341, 9.548067639142944], [-93.91437983942181, 9.477139513947902], [-98.23770276019286, 8.996702568213863], [-106.41185847396963, 8.221336475757616], [-120.34714917523388, 7.419047315902151], [-141.34174459902647, 7.127333352544687], [-165.3273044717421, 8.03669056104883], [-184.66859398195064, 10.409831377407066], [-197.42888229680392, 14.010955001081575], [0, 0])];
    BARREL_ROLL = [Path(-RobotMap.Drivetrain.max_auto_power,[2.1799171563768374, 10.112425339736726], [-2.250423374919633, 10.035507351676365], [-5.491058912282405, 10.017669994923441], [-7.5040575253054, 10.022744461231039], [-8.285408651077608, 10.028051462232687], [-7.837647913178556, 10.024781483426555], [-6.1590420484673984, 10.017982340839168], [-3.249787787894859, 10.026577369455802], [0.8649396645417698, 10.08317066742798], [6.097930354105848, 10.232761849870007], [12.260839154543733, 10.52913570225135], [19.048051988456233, 11.028096009343695], [26.32515316747138, 11.508295181218108], [35.551927179621686, 11.21210778488653], [49.42126590777986, 10.27089493381541], [71.51210048683265, 9.435593724256888], [-260.2836434316019, 9.952108825446235], [-238.77420840740479, 11.064889155991365], [-221.38770049960206, 11.43989643859809], [-201.92721522322594, 11.015427263788931], [-175.9880380796226, 10.669411301332634], [-151.85819533209806, 11.176518815238227], [-133.46024374024006, 11.417343082817645], [-115.38853905544076, 10.816794283415659], [-91.42532419360838, 9.74687169072201], [-64.15089564053795, 9.745109031907534], [-44.86281170865702, 10.661621964657073], [-32.13293662582085, 11.345986689971241], [-22.5153342757838, 11.26281919872835], [-13.713577833397956, 10.631126914717743], [-5.629640437538819, 10.230159727400423], [1.102660461650588, 10.06547966257416], [6.129996381459022, 10.033831526770133], [9.330201115257001, 10.052338212589902], [10.696254840454362, 10.067203893761207], [10.241136207789147, 10.055468697458473], [7.956011705588978, 10.02478687777695], [3.824106935094493, 10.01366327875581], [-2.1089346187359825, 10.091322086634433], [-9.619422761389046, 10.353223881340487], [-18.205954356275633, 10.900790145318979], [-27.316733298163633, 11.352516737768207], [-38.23912036210555, 11.024588672744047], [-53.85626450607808, 10.17489249600405], [-77.31438931419737, 9.623593007806825], [-104.0178299153778, 10.352620034686076], [-124.31472762566077, 11.226262291359536], [-142.70171653230219, 11.38300875418515], [-164.71640509496228, 11.078713041179062], [-191.5463229394581, 11.391819968152296], [-212.62905704738856, 12.003505268650377], [-230.09287098919697, 11.528220935123949], [-252.1662957217183, 9.865656065214848], [71.27054572657467, 8.707210071140917], [43.03345850616849, 10.177495212880794], [35.30974365080716, 11.253074903397026], [40.95715176028961, 10.588388217537519], [64.25350991533224, 9.139909354622855], [-269.206757465268, 9.711484922515288], [-263.95781965178054, 10.658953029045264], [84.12412922464925, 10.904472764489325], [65.39104153304311, 11.205742963849342], [48.291515122300154, 11.159329578999602], [29.539876177749846, 10.670982752056343], [5.49464261922653, 10.289958357215866], [-19.903505342599775, 10.819767122519094], [-39.05779113539583, 11.36880041194694], [-56.32814113012799, 11.142261869899187], [-77.48361803003033, 10.255261899189568], [-105.0004987525798, 9.86942933813513], [-127.3922525538633, 10.561394431449727], [-143.18238717843852, 11.196489406990143], [-156.25197346310802, 11.147544388744796], [-169.64121467909573, 10.435532431985177], [-180.6452053373737, 10.055488798583315], [-186.0214767060308, 10.010352789196116], [-185.95956140408313, 10.010368972484411], [-184.43602882954193, 10.014451386114413], [-183.04814997000474, 10.024344603163923], [-181.8009521417861, 10.038280882143248], [-180.69448062914265, 10.054670392828852], [-179.72822286882695, 10.072103795247115], [-178.90130988202955, 10.089353342253718], [-178.212689729167, 10.105371945909413], [-177.6612723108902, 10.119290999194616], [-177.24604633132972, 10.130417613597452], [-176.96617003025145, 10.138231801225464], [-176.8210375379074, 10.142384005785386], [-176.8103225589086, 10.142693273603188], [-176.93400067649947, 10.139146253733877], [-177.1923509895764, 10.131897122759531], [-177.58593713616733, 10.121268441263947], [-178.11556709274026, 10.107752860735117], [-178.7822305416977, 10.092015507242788], [-179.58701215193597, 10.074896767714522], [-180.53097891933857, 10.05741509329188], [-181.61503988874912, 10.0407693115997], [-182.8397772723128, 10.026339808655807], [-184.20524934942677, 10.015687809194057], [-185.71076772643008, 10.010551865090749], [0, 0])];
    BOUNCE = [Path(-RobotMap.Drivetrain.max_auto_power,[]),Path(RobotMap.Drivetrain.max_auto_power,[]),Path(-RobotMap.Drivetrain.max_auto_power,[]),Path(RobotMap.Drivetrain.max_auto_power,[])]

def calc_action(pt1, pt2):
    dx = pt2[0] - pt1[0]
    dy = pt2[1] - pt1[1]

    rad_ang = math.atan2(dy,dx)
    
    if dy < 0 and dx < 0:
        rad_ang+=2*math.pi

    angle = math.degrees(rad_ang)

    dist = math.sqrt(dx**2 + dy**2);

    return [angle,dist];

class Autonav(AutonomousStateMachine):
    MODE_NAME = "Autonav with PosApprox"

    drivetrain: Drivetrain
    location: PosApprox

    paths = Paths.SLALOM
    pathIndex = 0;
    actionIndex = 0;
    last_position = 0;
    currentPath = None;
    currentAction = [];

    @state(first=True)
    def track(self, initial_call):
        '''
        if initial_call:
            self.drivetrain.turn_to_angle(-150)
        elif self.drivetrain.turn_pid.get_on_target():
            self.drivetrain.stop()
            self.done()
        '''
        
        if initial_call:
            self.drivetrain.navx.reset()
            self.drivetrain.encoders.reset()
            self.location.reset();

            self.pathIndex = 0;
            self.currentPath = self.paths[self.pathIndex];
            
            self.actionIndex = -1; #ensures that it treats the previous n
            self.getNextAction(True);

            self.drivetrain.turn_to_angle(self.currentAction[0])

        else:
            direction = math.copysign(1,self.currentPath.power);
            if direction*(self.drivetrain.get_position(EncoderSide.BOTH)-self.last_position) > direction*self.currentAction[1]:
                self.getNextAction(True)
                self.drivetrain.turn_to_angle(self.currentAction[0])
                self.last_position = self.drivetrain.get_position(EncoderSide.BOTH)
                #print("node switch")

        self.drivetrain.powertrain.set_arcade_powers(power=self.currentPath.power)
    
    def getNextAction(self,usePosApprox):
        lastPos = [];
        if (usePosApprox):
            lastPos = self.location.get_location();
        else:
            lastPos = self.currentPath[self.actionIndex];
        self.actionIndex += 1;
        if (self.actionIndex >= len(self.currentPath)):
            self.pathIndex += 1;
            if self.pathIndex > len(self.paths):
                self.drivetrain.stop();
                self.done();
                return [0,0];
            self.currentPath = self.paths[self.pathIndex];
            self.actionIndex = 0;
        nextPos = self.currentPath[self.actionIndex];
        self.currentAction = calc_action(lastPos,nextPos);
        

        
        



            
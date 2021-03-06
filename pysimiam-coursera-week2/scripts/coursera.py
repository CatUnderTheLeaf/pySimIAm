#
# Testing solutions for Coursera
#
#
#
try:
  from urllib import urlencode
  from urllib2 import urlopen
except Exception:
  from urllib.parse import urlencode
  from urllib.request import urlopen
import hashlib
import base64
import re

class CourseraException(Exception):
    pass

class WeekTestCase:
    def __init__(self, week):
        self.testsuite = week
        self.name = "Name not set"
        self.test_id = "XXXYYYZZZ"

class WeekTest:
    coursera_challenge_url = "http://class.coursera.org/conrob-002/assignment/challenge"
    coursera_submit_url = "http://class.coursera.org/conrob-002/assignment/submit"

    def __init__(self, gui):
        self.gui = gui
        self.week = 0
        # Test is a tuple - 'title','id',function
        self.tests = []
        self.login = None
        self.password = None
        
        self.callback = None
        self.submit = True
        
        self.testname = 'Name not set'
    
    def setuser(self, login, password):
        self.login = str(login).strip()
        self.password = str(password).strip()
    
    def run_tests(self,tests = None):
        if tests is None:
            tests = list(range(len(self.tests)))
        for i in tests:
            self.test(self.tests[i])
    
    def test(self,testcase,callback):
        
        if isinstance(testcase,int):
            testcase = self.tests[testcase]
        
        self.callback = callback
        self.testcase = testcase

        params = urlencode({
            'email_address' : self.login,
            'assignment_part_sid' : testcase.test_id,
            'response_encoding' : 'delim'
            }).encode('utf-8')
        response = urlopen(url=WeekTest.coursera_challenge_url, data = params)

        string = response.read().decode('utf-8').split('|')[1:]

        self.c_response = {}
        for i in range(len(string)//2):
            try:
                self.c_response[string[2*i]] = string[2*i+1]
            except Exception:
                pass
            
        if 'email_address' not in self.c_response or not self.c_response['email_address']:
            raise CourseraException("Communication with server failed")
        elif 'challenge_key' not in self.c_response or not self.c_response['challenge_key'] \
                or 'state' not in self.c_response or not self.c_response['state']:
            # Error occured, error string in email_address
            raise CourseraException(self.c_response['email_address'])
        
        testcase.start_test(self.c_response['challenge_aux_data'])

    def respond(self,fn_output):
        
        if self.callback is None:
            return

        ch_resp = hashlib.sha1((self.c_response['challenge_key'] + self.password).encode('utf-8')).hexdigest()

        params = urlencode(
        {'assignment_part_sid': self.testcase.test_id,
            'email_address': self.c_response['email_address'],
            'submission': base64.standard_b64encode(fn_output.encode('utf-8')),
            'submission_aux': b'',
            'challenge_response': ch_resp,
            'state': self.c_response['state']}).encode('utf-8');
        
        self.callback(urlopen(url=self.coursera_submit_url, data = params).read().decode('utf-8'))
        
        self.testcase = None
        self.callback = None

class Week1(WeekTest):
  def __init__(self, gui):
    WeekTest.__init__(self, gui)
    
    self.testname = "Programming Assignment Week 1"
    
    self.week = 1
    self.tests.append(Week1Test1(self))
    
class Week1Test1(WeekTestCase):
    def __init__(self, week):
        self.testsuite = week
        self.name = "Running the simulator"
        self.test_id = "k3pa0rK4"

    def __call__(self,event,args):
        if event == "log":
            message, objclass, objcolor = args
            if message == "Switched to Hold":
                self.stop_test()
                self.testsuite.respond("-1")
        return False
        
    def start_test(self,challenge):
        self.testsuite.gui.start_testing()
        self.testsuite.gui.load_world('week1.xml')
        self.testsuite.gui.register_event_handler(self)
        self.testsuite.gui.run_simulation()
        
    def stop_test(self):
        self.testsuite.gui.unregister_event_handler()
        self.testsuite.gui.stop_testing()       

class Week2(WeekTest):
  def __init__(self, gui):
    WeekTest.__init__(self, gui)
    
    self.testname = "Programming Assignment Week 2"
    
    self.week = 2
    self.tests.append(Week2Test1(self))
    self.tests.append(Week2Test2(self))
    self.tests.append(Week2Test3(self))
    
class Week2Test1(WeekTestCase):
    
    RX = re.compile(r'v=(?P<v>[^;]+);w=(?P<w>[^;]+);')
    
    def __init__(self, week):
        self.testsuite = week
        self.name = "Unicycle to differential-drive\ntransformation"
        self.test_id = "QihGedxL"
        
    def start_test(self,challenge):
        m = self.RX.match(challenge)
        if m is None:
            raise CourseraException("Unknown challenge format. Please contact developers for assistance.")
        try:
            v = float(m.group('v'))
            w = float(m.group('w'))
        except ValueError:
            raise CourseraException("Unknown challenge format. Please contact developers for assistance.")
                    
        from supervisors.week2 import QuickBotSupervisor
        from robots.quickbot import QuickBot
        from pose import Pose
        
        info = QuickBot(Pose()).get_info()
        info.color = 0
        s = QuickBotSupervisor(Pose(),info)
        
        vl, vr = s.uni2diff((v,w))
        
        self.testsuite.respond("{:0.3f},{:0.3f}".format(vr,vl)) # Note the inverse order
        
class Week2Test2(WeekTestCase):
    
    RX = re.compile(r'v=(?P<v>[^;]+);theta_d=(?P<theta>[^;]+);')
    
    def __init__(self, week):
        self.testsuite = week
        self.name = "Odometry"
        self.test_id = "TQkrYtec"
        
    def start_test(self,challenge):
        m = self.RX.match(challenge)
        if m is None:
            raise CourseraException("Unknown challenge format. Please contact developers for assistance.")
        try:
            v = float(m.group('v'))
            theta = float(m.group('theta'))
        except ValueError:
            raise CourseraException("Unknown challenge format. Please contact developers for assistance.")
                    
        from supervisors.week2 import QuickBotSupervisor
        from robots.quickbot import QuickBot
        from pose import Pose
        from helpers import Struct
        from math import pi
        
        bot = QuickBot(Pose())
        info = bot.get_info()
        info.color = 0
        s = QuickBotSupervisor(Pose(),info)
        params = Struct()
        params.goal = theta*180/pi
        params.velocity = v
        params.pgain = 1
        s.set_parameters(params)
        
        tc = 0.033 # 0.033 sec' is the SimIAm time step
        
        for step in range(25): # 25 steps
            bot.move(tc)
            bot.set_inputs(s.execute(bot.get_info(), tc))
            
        xe,ye,te = s.pose_est
        xr,yr,tr = bot.get_pose()
        
        if xr == 0:
            xr = 0.0000001
        if yr == 0:
            yr = 0.0000001
        if tr == 0:
            tr = 0.0000001
        
        self.testsuite.respond("{:0.3f},{:0.3f},{:0.3f}".format(abs((xr-xe)/xr), abs((yr-ye)/yr), abs(abs(tr-te)%(2*pi)/tr)))

class Week2Test3(WeekTestCase):
    
    RX = re.compile(r'dist_1=(?P<d1>[^;]+);dist_2=(?P<d2>[^;]+);')
    
    def __init__(self, week):
        self.testsuite = week
        self.name = "Converting raw IR sensor values\nto distances"
        self.test_id = "yptGGVPr"
        
    def start_test(self,challenge):
        m = self.RX.match(challenge)
        if m is None:
            raise CourseraException("Unknown challenge format. Please contact developers for assistance.")
        try:
            d1 = float(m.group('d1'))
            d2 = float(m.group('d2'))
        except ValueError:
            raise CourseraException("Unknown challenge format. Please contact developers for assistance.")
                    
        from supervisors.week2 import QuickBotSupervisor
        from robots.quickbot import QuickBot, QuickBot_IRSensor
        from pose import Pose
        
        bot = QuickBot(Pose())
        sensor = QuickBot_IRSensor(Pose(),bot)
        
        id1 = sensor.distance_to_value(d1)
        id2 = sensor.distance_to_value(d2)
        
        info = bot.get_info()
        info.color = 0
        s = QuickBotSupervisor(Pose(),info)
        # Just in case a student iterates in a weird way
        s.robot.ir_sensors.readings = [id1,id2,id1,id2,id1]
        ird = s.get_ir_distances()
                    
        self.testsuite.respond("{:0.3f},{:0.3f}".format(abs((d1-ird[0])/d1), abs((d2-ird[1])/d2)))

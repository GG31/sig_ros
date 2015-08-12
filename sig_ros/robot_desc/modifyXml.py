
from lxml import etree
import sys
print str(sys.argv)
tree = etree.parse("/home/gg/catkin_ws/src/sig_ros/robot_desc/hiro.urdf")

def switchValues(attr):
    arg = attr.split()
    return arg[1] + " " + arg[2] + " " + arg[0]

#Switch axis
#for e in tree.findall('.//origin'):
#    e.attrib['xyz'] = switchValues(e.attrib['xyz'])
    #if 'rpy' in e.attrib:
    #    e.attrib['rpy'] = switchValues(e.attrib['rpy'])

#for e in tree.findall('.//axis'):
#    e.attrib['xyz'] = switchValues(e.attrib['xyz'])

#Add positions
jointsValues = {
            'base_footprint_joint': [0,0,0],
            
            #'torso_lift_joint': [0, 0, 0.836], # x of HEAD, y of LARM_JOINT0

            #'l_shoulder_pan_joint': [0, 0.1680, -0.004], #LARM_JOINT1
            #'l_shoulder_lift_joint': [0, 0, 0],
            #'l_upper_arm_roll_joint': [0,0,0],
            #'l_upper_arm_joint': [0,0,0],
            #'l_elbow_flex_joint': [0.1932,0,0],#LARM_JOINT4
            #'l_forearm_roll_joint': [0,0,0],
            #'l_forearm_joint': [0,0,0],
            #'l_wrist_flex_joint': [0.1312,0.0,0.0], #LARM_JOINT5
            #'l_wrist_roll_joint': [0,0,0],
            
            'torso_lift_joint': [sys.argv[1], sys.argv[2], sys.argv[3]], # x of HEAD, y of LARM_JOINT0

            'l_shoulder_pan_joint': [sys.argv[4], sys.argv[5], sys.argv[6]], #LARM_JOINT1
            'l_shoulder_lift_joint': [0, 0, 0],
            'l_upper_arm_roll_joint': [0,0,0],
            'l_upper_arm_joint': [0,0,0],
            'l_elbow_flex_joint': [sys.argv[7],sys.argv[8],sys.argv[9]],#LARM_JOINT4
            'l_forearm_roll_joint': [0,0,0],
            'l_forearm_joint': [0,0,0],
            'l_wrist_flex_joint': [sys.argv[10],sys.argv[11],sys.argv[12]], #LARM_JOINT5
            'l_wrist_roll_joint': [0,0,0],

            #'r_shoulder_pan_joint': [0,-0.1680,0], #RARM_JOINT1
            #'r_shoulder_lift_joint': [0, 0, 0],
            #'r_upper_arm_roll_joint': [0,0,0],
            #'r_upper_arm_joint': [0,0,0],
            #'r_elbow_flex_joint': [0.1932,0,0],#RARM_JOINT4
            #'r_forearm_roll_joint': [0,0,0],
            #'r_forearm_joint': [0,0,0],
            #'r_wrist_flex_joint': [0.1312,0.0,0.0], #RARM_JOINT5
            #'r_wrist_roll_joint': [0,0,0],
            
            'r_shoulder_pan_joint': [sys.argv[13],sys.argv[14],sys.argv[15]], #RARM_JOINT1
            'r_shoulder_lift_joint': [0, 0, 0],
            'r_upper_arm_roll_joint': [0,0,0],
            'r_upper_arm_joint': [0,0,0],
            'r_elbow_flex_joint': [sys.argv[16],sys.argv[17],sys.argv[18]],#RARM_JOINT4
            'r_forearm_roll_joint': [0,0,0],
            'r_forearm_joint': [0,0,0],
            'r_wrist_flex_joint': [sys.argv[19],sys.argv[20],sys.argv[21]], #RARM_JOINT5
            'r_wrist_roll_joint': [0,0,0],

        }

for elem in jointsValues:
    #a = tree.findall(".//link[@name='base_link']")
    #if len(a) == 1:
    #    a[0].find("origin").attrib['xyz'] = ' '.join(map(str, jointsValues['base_link']))
    a = tree.findall(".//joint[@name='" + elem + "']")
    #print str(len(a)) + " elem " + elem + " found"
    if len(a) == 1:
        #if 'xyz' not in a[0].find("origin").attrib:
            #print "pas de xyz pour " + elem
        a[0].find("origin").attrib['xyz'] = ' '.join(map(str, jointsValues[elem]))

with open("/home/gg/catkin_ws/src/sig_ros/robot_desc/out.urdf", 'w') as file_handle:
   file_handle.write(etree.tostring(tree, pretty_print=True, encoding='utf8'))

#!/usr/bin/env python

PACKAGE = 'hbrs_soundfile_player'
SERVICE = '~play_soundfile'

import roslib
roslib.load_manifest(PACKAGE)

import rospy
import hbrs_srvs.srv
import commands
import os


def play_soundfile_cb(request):
    rospy.loginfo('Received [%s] request.' % SERVICE)
        
    wav_path = commands.getoutput("rospack find raw_script_server")
    filename = wav_path + "/common/files/" + request.str

    empty_response = hbrs_srvs.srv.PassStringResponse()

    if not os.path.exists(filename):
        rospy.logerr("file \"%s\" does not exist", filename)
        return empty_response
        
    rospy.loginfo("Playing <<%s>>", filename)
    os.system("aplay -q " + filename)
    
    return empty_response


if __name__ == '__main__':
    rospy.init_node("hbrs_soundfile_player")
    
    srv_play_soundfile = rospy.Service(SERVICE, hbrs_srvs.srv.PassString, play_soundfile_cb)
    rospy.loginfo('Advertising service: %s' % SERVICE)
    rospy.loginfo('Node successfully initialized')
    rospy.spin()
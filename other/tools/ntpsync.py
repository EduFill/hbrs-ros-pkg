#!/usr/bin/env python
import ntplib
import os
import platform
import subprocess
import time 

DEFAULT_NTP_SERVER = "192.168.1.101"

if __name__ == "__main__":
	system = platform.system()
	try:
		server = os.environ["NTP_SERVER"] if "NTP_SERVER" in os.environ else DEFAULT_NTP_SERVER
		ntp_client = ntplib.NTPClient()
		response = ntp_client.request(server)
		
		time_value = time.ctime(response.tx_time)
		if system == "Linux":
			import syslog
			
			time_update_command = r"/bin/date"
			time_update_command_params = '--set="{0!s}"'.format(time_value)
			syslog.syslog("#"*20)
			syslog.syslog("Current time: {0!s}".format(time.ctime()))
			subprocess.check_call("{0!s} {1!s}".format(time_update_command, time_update_command_params), shell=True)
			syslog.syslog("NTP time sync from {0!s}: {1!s}".format(server, time_value))
			syslog.syslog("Current time: {0!s}".format(time.ctime()))
			syslog.syslog("#"*20)
		elif system == "Windows":
			time_update_command = r"time"
			time_update_command_params = time.strftime("%H:%M:%S", time.strptime(time_value))
			return_value = subprocess.call([time_update_command, time_update_command_params], shell=True)
			if return_value != 0:
				raise ValueError("Call to {0!s} returned error value {1!s}".format(time_update_command, return_value))
	except Exception, e:
		if system == "Linux":
			syslog.syslog(syslog.LOG_ERR, "NTP time sync failed: {0!s}".format(e))
		else:
			print e
	

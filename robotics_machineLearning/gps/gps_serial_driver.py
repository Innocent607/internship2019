import rospy
import serial 
import sys

def main():
	
	nmea_pub = rospy.Publish('nmea_sentences', Sentence, queue_size = 10)
	rospy.init_node('gps_serial_driver' anonymous = True)
	rate = rospy.Rate(1000)
	
	while not rospy.is_shutdown():
		nmea_sentences = GPS.readline()
			
		sentence = Sentence()
		sentence.header.stamp = rospy.get_rostime()
		sentence.header.frame_id = frame_id
		sentence.sentence = nmea_sentences
			
		rate.sleep()
@staticmethod
def get_param_id()
	
		
if __name__ == '__main__':

	port = rospy.get_param('~port', '/dev/ttyUSB0')
	baud = rospy.get_param('~baud', 9600)
	frame_id = rospy.get_param('~frame_id', 'gps')
	
	try:
		GPS = serial.Serial(port, baud, timeout = 2)
		
		try:
			main()
		
		except (rospy.ROSInterruptException, serial.serialutil.SerialException):
			GPS.close()
	
	except serial.SerialException as error:
		rospy.logfatal("Could not open serial port: I/O error({0}): {1}" .format(error.errno, error.strerror))
	
	
	
	

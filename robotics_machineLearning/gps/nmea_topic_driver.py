import rospy

def nmea_sentences_callback():



def main():
	rospy.init_node('nmea_topic_driver' anonymous = True)
	rospy.Subscriber('nmea_sentences', Sentence, nmea_sentences_callback)
	rospy.spin()

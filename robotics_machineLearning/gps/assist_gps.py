"""
U-blox A-GPS offerings:

AssistNow comes in two flavors: Online and Offline.
	-  Online is good for only two hours.
	-  Offline can last for a month. 
	
	u-blox requires an account to download the data
	 - send an email to {agps-account@u-blox.com}
	
"""
import socket
import serial

class gps:

	# We need to connect to Ublox- Server and we'll use socket package to download the data.
	#________________________________________CONNECTION_____________________________________

	def request_info():
		sock = socket.socket()

		# This information will be sent via e-mail.
		server_address = "agps.u-blox.com"
		server_port = 46434

		print ("connecting to u-blox")

		sock.connect((server_address, server_port))

		print("Successfully conneted: 100%")

		#______________________________________REQUESTING INFORMATION___________________________

		print ("please wait, sending your request")
		sock.send("cmd=full; user=xxx@xxx.xx; pwd=xxx; lat=50.0; lon=14.3; pacc=10000")

		data = ""
		buffer = 1;

		while buffer:
			buffer = sock.recv(1024)
	
			if buffer:
				data += buffer
				print(data)
			
		return data


	#________________________________________UPLOAD DATA 2 GPS_____________________________________
	def upload_2_gps():
		baudrate = 9600
		comm_port = "/dev/ttyAMA0"
		ser = serial.Serial(comm_port, baudrate)

		print("Looking for a freeline to send data")

		while True:
			drainer = ser.inWainting()
			ser.read(drainer)
	
		print ("Writing AGPS data")
		ser.write(binary)
		print ("Done")

	if __name__ == '__main__':
	
		request_info()
		headerEndsAt = data.index("\r\n\r\n")
		binaryStartsAt = headerEndsAt + 4 # length of the newline sequence
		binary = data[binaryStartsAt:]
		
		upload_2_gps()
		
		message = ""
		try:
			while True:
				buffer = ser.read()
				if buffer == "$":
				    if message.startswith("$GPGGA"):
				        print (message.strip())
				    message = ""
				message = message + buffer
		except KeyboardInterrupt:
			ser.close()

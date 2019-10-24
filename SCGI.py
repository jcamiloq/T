from time import sleep, clock
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import datetime
import math
from pymavlink import mavutil
import serial
import threading

COM = 'COM1'
Modo_foto=""
Valor_modo=10.2
a = 0
exclu=1
bandFoto=0
cnt=0
GPS_FOTO=[]
TIME_FOTO=[]
vehicle=""
point1=""
point2=""
point3=""
point4=""
altura=""
points_lat=[]
points_long=[]
DataWps=[]
band_pause=0
band_pause_2=0
band_cancel=0
acabe=0
band_RTH=0
band_cancel_2=0
flag_reanudado=0
Mision_resume=0
band_battery=0
nxt_Wp=0;
fin=0
RTH_fin=0
ActualWp=0
nextwaypoint=1
Wp_reanudacion=0
lati_r=""
longi_r=""
velocidad=""
estado='c'

#Set up option parsing to get connection string
import argparse  
'''
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

reanudar=argparse.ArgumentParser(description='Comandos de reanudacion')
reanudar.add_argument('--reanudar',help="Posicion GPS almacenada por ultima vez por el dron")

Ulti_GPS=reanudar.parse_args()

connection_string = args.connect
sitl = None
'''
parser = argparse.ArgumentParser()
parser.add_argument("--connect", dest="conect",help="Puerto por el que se desea conectar el UAV Ej: Com7")

parser.add_argument("--reanudar",dest="reanude",help="Valores necesarios para realizar la reanudacion con el siguiente formato : [LATITUD,LONGITUD,WP_REANUDACION]")

results = parser.parse_args()
connection_string=results.conect
sitl=None
Ulti_GPS=results.reanude
print "Ulti %s" %Ulti_GPS
print "Cone %s" %connection_string

if not Ulti_GPS:
	print "START MISION"
	global flag_reanudado
	flag_reanudado=0
else:
	print "START MISION REANUDED"
	
	global flag_reanudado
	flag_reanudado=1
	pos=Ulti_GPS
	Posicion_reanudar=pos.split("/")
	global lati_r
	global longi_r
	global Wp_reanudacion
	lati_r=Posicion_reanudar[0]
	lati_r=lati_r[1:]
	lati_r=float(lati_r)
	longi_r=float(Posicion_reanudar[1])
	Wp_reanudacion=Posicion_reanudar[2]
	cnt= Posicion_reanudar[3]
	cnt=cnt[:-1]
	cnt=int(cnt)
	Wp_reanudacion=int(Wp_reanudacion)
	print"Longitud  de reanudacion: %s" %longi_r
	print"Latitud  de reanudacion: %s" %lati_r
	print"Wpoint  de reanudacion: %s" %Wp_reanudacion
	print"Contador fotos: %s" %cnt

'''
#Start SITL if no connection string specified // ESTO DEBE SER CAMBIADO POR CONNECCION CON COM 
import dronekit_sitl
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()
'''
if not connection_string:
	print "Empezo con simulador"
	import dronekit_sitl
	sitl = dronekit_sitl.start_default()
	connection_string = sitl.connection_string()

def iniComunicaSerial (COM):
		portCOM = serial.Serial()
		portCOM.baudrate = 9600
		portCOM.port = COM
		portCOM.bytesize = serial.EIGHTBITS
		portCOM.parity = serial.PARITY_NONE
		portCOM.stopbits = serial.STOPBITS_ONE
		portCOM.timeout = 1
		portCOM.xonxoff = False
		portCOM.rtscts = False
		portCOM.dsrdtr = False
		
		try:
			portCOM.open()
		except Exception, e:
			print "Error serial port: " + str(e)
			exit()
		
		return portCOM

iniComunicaSerial(COM)
portCOMx = iniComunicaSerial(COM)

def abrir_log(nombre):
    
    archivo_log = open(nombre, "a")
    return archivo_log
 
def guardar_log(archivo_log,mensaje,tipo):
   
    # Obtiene la hora actual en formato de texto
    hora_actual = str(datetime.datetime.now())
    # Guarda la hora actual y el mensaje de error en el archivo
    archivo_log.write("\n"+tipo+": "+hora_actual+" "+"DistanciaHome: "+mensaje)
 
def cerrar_log(archivo_log):
    
    archivo_log.close()

def get_location_metres(original_location, dNorth, dEast):
	"""
	Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
	specified `original_location`. The returned Location has the same `alt` value
	as `original_location`.

	The function is useful when you want to move the vehicle around specifying locations relative to 
	the current vehicle position.
	The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
	For more information see:
	http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
	"""
	earth_radius=6378137.0 #Radius of "spherical" earth
	#Coordinate offsets in radians
	dLat = dNorth/earth_radius
	dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

	#New position in decimal degrees
	newlat = original_location.lat + (dLat * 180/math.pi)
	newlon = original_location.lon + (dLon * 180/math.pi)
	return LocationGlobal(newlat, newlon,original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
	"""
	Returns the ground distance in metres between two LocationGlobal objects.

	This method is an approximation, and will not be accurate over large distances and close to the 
	earth's poles. It comes from the ArduPilot test code: 
	https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
	"""
	dlat = aLocation2.lat - aLocation1.lat
	dlong = aLocation2.lon - aLocation1.lon
	return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint():
	"""
	Gets distance in metres to the current waypoint. 
	It returns None for the first waypoint (Home location).
	"""
	nextwaypoint = vehicle.commands.next
	if nextwaypoint==0:
		return None
	missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
	lat = missionitem.x
	lon = missionitem.y
	alt = missionitem.z
	targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
	distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
	return distancetopoint
	
def distance_to_someone_waypoint(Lati1,Longi1):
	targetWaypointLocation = LocationGlobalRelative(Lati1, Longi1,altura)
	distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
	return distancetopoint

def SCGI_mission (ArrayList, Altura_wps):
	cmds = vehicle.commands

	print " Clear any existing commands"
	cmds.clear() 

	print " Define/add new commands."
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, Altura_wps))
	
	for Wps in ArrayList:
		# Add new commands. The meaning/order of the parameters is documented in the Command class. 
	 	#Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
		pointsW=Wps.split(",")
		cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, float(pointsW[0]), float(pointsW[1]),  Altura_wps))
	#add dummy waypoint "n" at point n-1 (lets us know when have reached destination)
	LastData=len(ArrayList)-1
	lastpoint=ArrayList[LastData]
	sep=lastpoint.split(",")
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, vehicle.home_location.lat,vehicle.home_location.lon, 0))	
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, vehicle.home_location.lat, vehicle.home_location.lon,  Altura_wps))	
	print " Upload new commands to vehicle"
	cmds.upload()

def adds_square_mission(aLocation, aSize):
	"""
	Adds a takeoff command and four waypoint commands to the current mission. 
	The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

	The function assumes vehicle.commands matches the vehicle mission state 
	(you must have called download at least once in the session and after clearing the mission)
	"""	

	cmds = vehicle.commands

	print " Clear any existing commands"
	cmds.clear() 

	print " Define/add new commands."
	# Add new commands. The meaning/order of the parameters is documented in the Command class. 
	 
	#Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
	#cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
	#Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
	global point1
	global point2
	global point3
	global point4
	point1 = get_location_metres(aLocation, aSize, -aSize)
	point2 = get_location_metres(aLocation, aSize, aSize)
	point3 = get_location_metres(aLocation, -aSize, aSize)
	point4 = get_location_metres(aLocation, -aSize, -aSize)
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
	#add dummy waypoint "5" at point 4 (lets us know when have reached destination)
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))    

	print " Upload new commands to vehicle"
	cmds.upload()			
	
def worker():
	"""funcion que realiza el trabajo en el thread"""
	portCOMx.write("Q")
	while True:
		global cnt
		dataInmatlab = portCOMx.readline()
		dataInmatlab=str(dataInmatlab)
		if dataInmatlab=="X":
			while True:
				dataInmatlab = portCOMx.readline()
				dataInmatlab=str(dataInmatlab)
				print "Dato de java: %s" %dataInmatlab
				global fin
				portCOMx.write("Z")
				if dataInmatlab=="R":
					bateria_datos=str(vehicle.battery)
					bateria_datos_array=bateria_datos.split(",")
					Voltaje=bateria_datos_array[0]
					Voltaje=Voltaje[16:]
					a=vehicle.location.global_frame
					b=vehicle.attitude
					c=vehicle.location.global_relative_frame.alt
					d=vehicle.airspeed
					f=vehicle.mode
					portCOMx.write(str(a)+","+str(b)+","+str(c)+","+str(d)+","+str(Voltaje)+","+str(cnt)+","+str(time.strftime("%H:%M:%S"))+","+str(f)+","+str(nextwaypoint))
					portCOMx.write("X")
				if fin==1:
					global acabe
					print"Informacion de finalizado"
					portCOMx.write(str("FIN DE MISION"))
					portCOMx.write("X")
					time.sleep(2)
					portCOMx.write("N")
					for x in GPS_FOTO:
						print"Envie %s" %x
						portCOMx.write(x+"/")
					portCOMx.write("P")
					acabe=1
				if RTH_fin==1:
					portCOMx.write(str("MISION RTH"))
					portCOMx.write("X")
					
				if band_pause_2==1:
					global acabe
					print"Informacion de pausado"
					portCOMx.write(str("MISION PAUSADA"))
					portCOMx.write("X")
					time.sleep(2)
					portCOMx.write("N")
					for x in GPS_FOTO:
						print"Envie %s" %x
						portCOMx.write(x+"/")
					portCOMx.write("P")
					acabe=1
				if band_cancel_2==1:
					portCOMx.write(str("MISION CANCELADA"))
					portCOMx.write("X")
				if Mision_resume==1:
					global Mision_resume
					portCOMx.write(str("CONTINUA MISION"))
					portCOMx.write("X")
					Mision_resume=0
				
				if band_battery==1:
					Pos_wait=vehicle.location.global_frame 
					global band_battery
					global bandFoto
					bandFoto=1
					portCOMx.write(str("BATERIA"))
					portCOMx.write("X")
					band_battery=0
				
				if dataInmatlab=="TR" or dataInmatlab=="T" or dataInmatlab=="RT":
					global nextwaypoint
					global cnt
					print "Enviando datos de pausa.."
					portCOMx.write("N"+str(Pos_wait)+","+str(nextwaypoint)+","+str(cnt))
					portCOMx.write("P")
				if dataInmatlab=="RW":
					print "PAUSAR"
					global band_pause
					global bandFoto
					bandFoto=1
					band_pause=1
					Pos_wait=vehicle.location.global_frame 
					portCOMx.write(str("PAUSA"))
				if dataInmatlab=="RV":
					print "CANCELAR"
					global bandFoto
					bandFoto=1
					global band_cancel
					band_cancel=1
				if dataInmatlab=="RH":
					print "IR A CASAAAA !!"
					global band_RTH
					global bandFoto
					bandFoto=1
					band_RTH=1

def Camera():
	print "INICIO HILO TOMA DE FOTOS"
	global a
	global b
	global bandFoto
	global Valor_modo
	global vehicle
	global cnt
	global GPS_FOTO
	global TIME_FOTO
	global WpPar
	while True:
		
		if (nextwaypoint>1 and WpPar==0 ):
			b = clock()
			if bandFoto == 1:
				print"FIN TOMA DE FOTOS"
				break
			elif b - a > Valor_modo:
				print "Cada %s segundos, se esta ejecutando el hilo1" %Valor_modo
				msg = vehicle.message_factory.digicam_control_encode(0,0,0,0,0,0,1,0,0,0)
				vehicle.send_mavlink(msg)
				Pos2=[]
				Pos=str(vehicle.location.global_frame)
				Pos2=Pos.split("=")
				GPS_FOTO.append(Pos2[1]+Pos2[2]+str(time.strftime("%H:%M:%S")))
				cnt=cnt+1
				a = b
		if (WpPar!=0):
			b=clock()
			a=0
										

	
def main ():	
	'''
	Variable global 
	'''
	 
	'''
	#Estados 
	'''
	def EDOc(entrada):
		global estado 
		global vehicle 
		while True:
			dataInmatlab = portCOMx.readline()
			dataInmatlab=str(dataInmatlab)
			print "Esperando la conexion: %s" %dataInmatlab
			if dataInmatlab=="C":
				print 'Connecting to vehicle on: %s' % connection_string
				
				vehicle = connect(connection_string,baud=57600,wait_ready=True)	
				'''
				vehicle= connect(connection_string,wait_ready=True)
				'''
				d=vehicle.location.global_frame 
				f=vehicle.version
				g=vehicle.battery
				h=vehicle.gps_0
				print("%s") %vehicle.gps_0
				print("%s") %vehicle.battery
				portCOMx.write("F")
				portCOMx.write(str(d)+","+str(f)+","+str(g)+","+str(h))
				portCOMx.write("Z")
				
				estado='i'
				break
			
			
	def EDOi(entrada): 
		global estado 
		print"Estado Inicial" 
		hola ="pitch=5"
		parametros =""			
				
		portCOMx.write("H")
		while True:
			dataInmatlab = portCOMx.readline()
			dataInmatlab=str(dataInmatlab)
			print "Dato de java: %s" %dataInmatlab
			if dataInmatlab=="X":
				print "Start Flag"
				portCOMx.write("L")
			if dataInmatlab!="X" and dataInmatlab!="Z" and dataInmatlab!="":
				parametros+=dataInmatlab
				print "Parameters= %s" %parametros
				portCOMx.write("M")
			if dataInmatlab=="Z":
				al_vel=parametros.split(",")
				global altura
				global velocidad
				global Modo_foto
				global Valor_modo
				altura=al_vel[0]
				velocidad=al_vel[1]
				Modo_foto=al_vel[2]
				Valor_modo=al_vel[3]
				altura=float(altura)
				velocidad=float(velocidad)
				Valor_modo=float(Valor_modo)
				print "Altura: %s" %altura
				print "Velocidad: %s" %velocidad
				print "Modo foto: %s" %Modo_foto
				print "Valor_modo: %s" %Valor_modo
				print "End Flag"
				break
			
		estado = 'j'
		print "Transicion hacia j..."			
		
		

	def EDOj(entrada): 
		global estado 
		global points_lat
		global points_long
		global DataWps
		print"Estado j"
		arregloWp=[]
		hola ="pitch=5"
		parametros =""			
		portCOMx.write("O")
		while True:
			dataInmatlab = portCOMx.readline()
			dataInmatlab=str(dataInmatlab)
			print "Dato de java: %s" %dataInmatlab
			if dataInmatlab=="X":
				print "Start Flag"
				portCOMx.write("L")
			if dataInmatlab!="X" and dataInmatlab!="Z" and dataInmatlab!="":
				dato=dataInmatlab
				arregloWp.append(dato)
				print "Dato= %s" %dato
				portCOMx.write("M")
			if dataInmatlab=="Z":				
				DataWps=arregloWp
				points_lat=[]
				points_long=[]
				for x in arregloWp:					
					print "cada wp: %s" %x
					points=x.split(",")
					points_lat.append(float(points[0]))
					points_long.append(float(points[1]))
					#print "cada wp lat: %s" %points[0]
					#print "cada wp long: %s " %points[1]
				print "End Flag"
				break
		
				
		estado = 0
		print "Transicion hacia 0..."			
		
		
		

			
	def EDO0(entrada): 
		global estado 
		print"Estado 0"
		
		#print "Arreglo longs: %s" %points_long
		global vehicle 
	
		vehicle.home_location = vehicle.location.global_frame
		print " New Home Location (from attribute - altitude should be 222): %s" % vehicle.home_location
		
		# We have a home location, so print it!        
		print "\n Home location: %s" % vehicle.home_location
		
			
		print "Arming motors"
		# Copter should arm in GUIDED mode
		vehicle.mode = VehicleMode("STABILIZE")
		vehicle.armed = True    

		# Confirm vehicle armed before attempting to take off
		while not vehicle.armed:      
			print " Waiting for arming..."
			time.sleep(1)
		
			
		t1 =threading.Thread(target = worker)
		t1.setDaemon(True)
		t1.start()
		
		'''
		Transiciones 
		'''
		sleep(2) 
		if entrada == 0: 
			estado = 0 
			print"Transicion hacia 0..." 
		if entrada == 1: 
			estado = 1 
			print"Transicion hacia 1..."
		if entrada == 2:
			estado == 1
			print"Transicion hacia 1..." 
			 
	def EDO1(entrada): 
		global estado 
		print"Estado 1"
		#adds_square_mission(vehicle.location.global_frame,50)
		print "Taking off!"
		vehicle.mode = VehicleMode("GUIDED")
		vehicle.simple_takeoff(altura) # Take off to target altitude
		# Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
		#  after Vehicle.simple_takeoff will execute immediately).
		
		while True:
			print " Altitude: ", vehicle.location.global_relative_frame.alt
			#Break and return from function just below target altitude.        
			if vehicle.location.global_relative_frame.alt>=altura*0.95: 
				print "Reached target altitude"
				Wpoint=0
				break
			time.sleep(1)

		print "Set default/target airspeed to %s" %velocidad
		vehicle.airspeed = velocidad
		
				
		'''
		Transiciones 
		''' 
		if flag_reanudado==1:
			estado='r'
			print"Transicion hacia r..."
			time.sleep(5)
		else:
			if entrada == 1: 
				estado = 'm' 
				print"Transicion hacia m..."
				time.sleep(5)

			
	def EDOm(entrada):
		global estado
		global nxt_Wp
		global WpPar
		global bandFoto
		global cnt
		global band_battery
		print"Estado m"	
		print "Starting mission"
		
		#adds_square_mission(vehicle.location.global_frame,50)
		SCGI_mission(DataWps,altura)
		
		vehicle.commands.next=nxt_Wp
		
		# Set mode to AUTO to start mission
		vehicle.mode = VehicleMode("AUTO")


		# Monitor mission. 
		# Demonstrates getting and setting the command number 
		# Uses distance_to_current_waypoint(), a convenience function for finding the 
		#   distance to the next waypoint.

		n=2
		bande=1
		bande2=1
		bande3=1
		while True:
			global exclu
			global band_pause
			global band_cancel
			global nextwaypoint
			global ActualWp
			nextwaypoint=vehicle.commands.next
			WpPar=nextwaypoint%2
			print "PAR O IMPAR %s" %WpPar
			bateria_datos=str(vehicle.battery)
			bateria_datos_array=bateria_datos.split(",")
			Voltaje=bateria_datos_array[0]
			Level=bateria_datos_array[2]
			Voltaje=Voltaje[16:]
			Level=Level[6:]
			print 'Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint())
			print " Altitude: ", vehicle.location.global_relative_frame.alt   
			if band_pause==1:
				print "Exit 'standard' mission for PAUSED button"
				ActualWp=nextwaypoint-1
				estado = 'p'
				print"Transicion hacia p..."
				break;
			if band_RTH==1:
				print "Exit 'standard' mission for RTH button"
				estado = 3 
				print"Transicion hacia 3..."
				break;
			if band_cancel==1:
				print "Exit 'standard' mission for CANCEL button"
				estado= 'h'
				print"Transicion hacia h..."
				break;
			if float(Voltaje)<=9.0 and nextwaypoint<int(len(DataWps)+1):
				print "Exit 'standard' mission for BATTERY"
				ActualWp=nextwaypoint-1
				band_battery=1
				estado = 'p'
				print"Transicion hacia p..."
				break;
			if Modo_foto=="0":
				print"MODO DIST: Valor_modo:%s" %Valor_modo
				if bandFoto==0:
					if WpPar==0 and nextwaypoint>1:
						if bande==1 and nextwaypoint>1:
							Pye=float(distance_to_current_waypoint()) - (Valor_modo)
							print"Actualiza Pye=%s N=%s" %(Pye,n)
							if bande2==1:
								bande=0
								print("ya se actualizo y salgo")
							if nextwaypoint==n and bande3==1:
								'''
								msg = vehicle.message_factory.digicam_control_encode(0,0,0,0,0,0,1,0,0,0)
								vehicle.send_mavlink(msg)
								Pos2=[]
								Pos=str(vehicle.location.global_frame)
								Pos2=Pos.split("=")
								GPS_FOTO.append(Pos2[1]+Pos2[2]+str(time.strftime("%H:%M:%S")))
								cnt=cnt+1
								print"TOME_FOTO_1: %s" %distance_to_current_waypoint()
								'''
								bande2=1
								bande=0
						if nextwaypoint>1 and bande2==1:
							print"PYE: %s" %Pye
							if distance_to_current_waypoint()<=Pye:
								msg = vehicle.message_factory.digicam_control_encode(0,0,0,0,0,0,1,0,0,0)
								vehicle.send_mavlink(msg)
								Pye=float(distance_to_current_waypoint()) - (Valor_modo)
								Pos2=[]
								Pos=str(vehicle.location.global_frame)
								Pos2=Pos.split("=")
								GPS_FOTO.append(Pos2[1]+Pos2[2]+str(time.strftime("%H:%M:%S")))
								cnt=cnt+1
								print"TOME_FOTO_2: %s" %distance_to_current_waypoint()
								bande3=0
								
							else :
								if distance_to_current_waypoint()<=Valor_modo:
									print"valor menor para toma de foto"
									if bande2==1:
										print"condicional menor"
										bande=1
										bande3=1
										n=n+1
										bande2=0
					else:
						print"No es par el wp"
						Pye=0
						bande=1
						bande2=1
						bande3=1
				else:
					print"FIN TOMA DE DATOS"
							
			if Modo_foto=="1" and exclu==1:
				print"Modo tiempo"
				t2 =threading.Thread(target = Camera)
				t2.setDaemon(True)
				t2.start()
				exclu=0
			if (nextwaypoint==int(len(DataWps)+1)):
				bandFoto=1
			if (nextwaypoint==int(len(DataWps)+1)) and (distance_to_current_waypoint()<=1): #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
				print "Exit 'standard' mission when start heading to FINAL waypoint "
				bandFoto=1
				estado = 2 
				print"Transicion hacia 2..."
				break;
			
					

	def EDOr(entrada):
		global estado
		global nxt_Wp
		global Mision_resume
		vehicle.mode = VehicleMode("GUIDED")
		print "Going towards RESUME point  ..."
		pointy = LocationGlobalRelative(lati_r, longi_r, altura)
		vehicle.simple_goto(pointy)
		while True:
		# sleep so we can see the change in map
			Distancia=distance_to_someone_waypoint(pointy.lat,pointy.lon)
			print"Distancia a reanudacion: %s" %Distancia
			print"altitud de retorno: ", vehicle.location.global_relative_frame.alt
			if((Distancia<=1)):
				print"Punto reanudacion alcanzado!!!!"
				Mision_resume=1
				print "Wp de reanudacion: %s" %Wp_reanudacion
				nxt_Wp=int(Wp_reanudacion)
				estado='m'
				print"Transicion hacia m..."
				break
			if band_RTH==1:
				print "Exit 'standard' mission for RTH button"
				estado = 3 
				print"Transicion hacia 3..."
				break;
			if band_cancel==1:
				print "Exit 'standard' mission for CANCEL button"
				estado= 'h'
				print"Transicion hacia h..."
				break;
		
	def EDO3(entrada): 
		global estado 
		archivo_log=abrir_log("RTL_monitoreo.txt")
		guardar_log(archivo_log,str(distance_to_someone_waypoint(vehicle.home_location.lat,vehicle.home_location.lon)),"RTL")
		cerrar_log(archivo_log)
		print"Estado 3"
		print "Returning to Launch"
		#vehicle.mode = VehicleMode("RTL")
		vehicle.mode = VehicleMode("GUIDED")
		
		pointy = LocationGlobalRelative(float(str(vehicle.home_location.lat)),float(str(vehicle.home_location.lon)), altura)
		vehicle.simple_goto(pointy)
		while True:
			print"Yendo a casa..."
			Distancia=distance_to_someone_waypoint(vehicle.home_location.lat,vehicle.home_location.lon)
			print"Distancia a casa: %s" %Distancia
			print"altitud de retorno: ", vehicle.location.global_relative_frame.alt
			if((Distancia<=2)):
				print"EN CASA!!!!"
				break;
			time.sleep(0.5)
		
		vehicle.mode = VehicleMode("LAND")
		while True:
			print"Distancia a casa: %s" %Distancia
			print"altitud de retorno: ", vehicle.location.global_relative_frame.alt
			#Break and return from function just below target altitude.        
			if vehicle.location.global_relative_frame.alt<=1: 
				print "Altitud de descenso alcanzada"
				Wpoint=0
				break
			time.sleep(0.5)
				
		time.sleep(2)		
		print "Global Location: %s" % vehicle.location.global_frame 
		#Close vehicle object before exiting script
		global fin
		global RTH_fin
		if band_RTH!=1:
			fin=1
		else:
			RTH_fin=1
		#vehicle.mode = VehicleMode("GUIDED")
		vehicle.armed = False
		while vehicle.armed:
			print " Waiting for disarming.."
			print "El vehiculo armed(true) o disarmed (false): %s" %vehicle.armed
			time.sleep(1)
		print "Close vehicle object"
		vehicle.close()
		# Shut down simulator if it was started.
		if sitl is not None:
			sitl.stop()
		'''
		Transiciones 
		''' 
		estado = 'f' 
		print"Transicion hacia f..." 
	
	
	def EDO2(entrada): 
		global estado 
		print"Estado 2"
		print "Returning to Launch"
		#vehicle.mode = VehicleMode("RTL")
		while True:
			print"altitud de retorno: ", vehicle.location.global_relative_frame.alt
			#Break and return from function just below target altitude.        
			if vehicle.location.global_relative_frame.alt<=1: 
				print "Altitud de descenso alcanzada"
				Wpoint=0
				break
			time.sleep(0.5)
				
		time.sleep(2)
		print "Global Location: %s" % vehicle.location.global_frame 
		#Close vehicle object before exiting script
		global fin
		global RTH_fin
		if band_RTH!=1:
			fin=1
			print"FINALIZO"
		else:
			RTH_fin=1
		vehicle.mode = VehicleMode("STABILIZE")
		vehicle.armed = False
		while vehicle.armed:
			print " Waiting for disarming.."
			print "El vehiculo armed(true) o disarmed (false): %s" %vehicle.armed
			time.sleep(1)
		while True:
			global acabe
			print"pausa activa"
			if acabe==1:
				break
		print "Close vehicle object"
		vehicle.close()
		
		# Shut down simulator if it was started.
		if sitl is not None:
			sitl.stop()
		'''
		Transiciones 
		''' 
		estado = 'f' 
		print"Transicion hacia f..." 
		 
	
	def EDOf(entrada): 
		global estado 
		print"Estado f"
		
	
	def EDOh(entrada): 
		global estado
		global band_cancel_2
		archivo_log=abrir_log("Cancel_monitoreo.txt")
		guardar_log(archivo_log,str(distance_to_someone_waypoint(vehicle.home_location.lat,vehicle.home_location.lon)),"CANCELA")
		cerrar_log(archivo_log)
		print"Estado h"
		vehicle.mode = VehicleMode("LAND")
		while True:
			print " Altitude: ", vehicle.location.global_relative_frame.alt
			#Break and return from function just below target altitude.        
			if vehicle.location.global_relative_frame.alt<=2: 
				print "Altitud de descenso alcanzada"
				break
			time.sleep(0.5)
		
		time.sleep(5)
		band_cancel_2=1
		#vehicle.mode = VehicleMode("GUIDED")
		vehicle.armed = False
		while vehicle.armed:
			print " Waiting for disarming.."
			print "El vehiculo armed(true) o disarmed (false): %s" %vehicle.armed
			time.sleep(1)
		print "Close vehicle object"
		vehicle.close()
		# Shut down simulator if it was started.
		if sitl is not None:
			sitl.stop()
		print "CANCELACION REALIZADA Y EN TIERRA"
		
		
			
	def EDOp(entrada): 
		global estado 
		global band_pause_2
		print"Estado p"
		archivo_log=abrir_log("Pausa_monitoreo.txt")
		guardar_log(archivo_log,str(distance_to_someone_waypoint(vehicle.home_location.lat,vehicle.home_location.lon)),"PAUSA")
		cerrar_log(archivo_log)
		vehicle.mode = VehicleMode("GUIDED")
	
		if(ActualWp>=1):
			Distancia_ant=distance_to_someone_waypoint(points_lat[ActualWp-1],points_long[ActualWp-1])
			Distancia_sig=distance_to_someone_waypoint(points_lat[ActualWp],points_long[ActualWp])
			if(Distancia_ant<=Distancia_sig):
				puntoc=LocationGlobalRelative(points_lat[ActualWp-1],points_long[ActualWp-1],altura)
				ps=1
				print"Va al anterior"
			else:
				puntoc=LocationGlobalRelative(points_lat[ActualWp],points_long[ActualWp],altura)
				print"Va al siguiente"
				ps=0
			vehicle.simple_goto(puntoc)
			
			while True:
				if ps==1:
					Distancia=distance_to_someone_waypoint(points_lat[ActualWp-1],points_long[ActualWp-1])
				else:
					Distancia=distance_to_someone_waypoint(points_lat[ActualWp],points_long[ActualWp])
				print"Distancia a Wp para bajar: %s" %Distancia
				print"altitud : ", vehicle.location.global_relative_frame.alt
				if((Distancia<=3)):
					print"Punto reanudacion alcanzado!!!!"
					break;
				time.sleep(0.5)
			
			print"Llego al punto de bajar"
			vehicle.mode = VehicleMode("LAND")
			while True:
				print " Altitude: ", vehicle.location.global_relative_frame.alt
				#Break and return from function just below target altitude.        
				if vehicle.location.global_relative_frame.alt<=2: 
					print "Altitud de descenso alcanzada"
					Wpoint=0
					break
				time.sleep(0.5)
				
			time.sleep(5)
			band_pause_2=1
			#vehicle.mode = VehicleMode("GUIDED")
			vehicle.armed = False
			while vehicle.armed:
				print " Waiting for disarming.."
				print "El vehiculo armed(true) o disarmed (false): %s" %vehicle.armed
				time.sleep(1)
			while True:
				global acabe
				print"pausa activa"
				if acabe==1:
					break
			print "Close vehicle object"
			vehicle.close()
			
			# Shut down simulator if it was started.
			if sitl is not None:
				sitl.stop()
			
		else:
			vehicle.mode = VehicleMode("GUIDED")
			pointy = LocationGlobalRelative(float(str(vehicle.home_location.lat)),float(str(vehicle.home_location.lon)), altura)
			vehicle.simple_goto(pointy)
			while True:
				print"Yendo a casa..."
				Distancia=distance_to_someone_waypoint(vehicle.home_location.lat,vehicle.home_location.lon)
				print"Distancia a casa: %s" %Distancia
				print"altitud de retorno: ", vehicle.location.global_relative_frame.alt
				if((Distancia<=2)):
					print"EN CASA!!!!"
					break;
				time.sleep(0.5)
			vehicle.mode = VehicleMode("LAND")
			while True:
				print"Distancia a casa: %s" %Distancia
				print"altitud de retorno: ", vehicle.location.global_relative_frame.alt
				#Break and return from function just below target altitude.        
				if vehicle.location.global_relative_frame.alt<=1: 
					print "Altitud de descenso alcanzada"
					Wpoint=0
					break
				time.sleep(0.5)
	
			band_pause_2=1
			#vehicle.mode = VehicleMode("GUIDED")
			vehicle.armed = False
			while vehicle.armed:
				print " Waiting for disarming.."
				print "El vehiculo armed(true) o disarmed (false): %s" %vehicle.armed
				time.sleep(1)
			print "Close vehicle object"
			vehicle.close()
			# Shut down simulator if it was started.
			if sitl is not None:
				sitl.stop()
		print "VEHICULO PAUSADO Y EN TIERRA"
			
	'''
	#Finite State Machine (FSM)
	'''    
	def FSM(entrada): 
		global estado 
		switch = {
			'c' :EDOc,
			'i':EDOi,
			'j':EDOj, 
			0 :EDO0, 
			1 :EDO1, 
			2 :EDO2,
			3 :EDO3,
			'm':EDOm,
			'r':EDOr,
			'h':EDOh,
			'f':EDOf,
			'p':EDOp,
		} 
		func = switch.get(estado, lambda: None) 
		return func(entrada) 
	
	
	'''
	#Programa Principal 
	'''
	if flag_reanudado==0:
		print"FSM normal"
		FSM(1)
		sleep(.3)
		while True:
			dataInmatlab = portCOMx.readline()
			dataInmatlab=str(dataInmatlab)
			print "Esperando INICIO: %s" %dataInmatlab
			if dataInmatlab=="I":
				print "Inicio de mision correcto"
				break
			time.sleep(0.2)
		FSM(1)
		sleep(.3)
		FSM(1)
		sleep(.3)
		print "check"
		FSM(1)
		sleep(.3) 
		print "check2"
		FSM(1)
		sleep(.3) 
		FSM(1)  
		sleep(.3)
		FSM(0)  
		sleep(2)
		#Completo
	else:
		print"FSM reanudacion"
		FSM(1)
		sleep(.3)
		'''
		while True:
			dataInmatlab = portCOMx.readline()
			dataInmatlab=str(dataInmatlab)
			print "Esperando REANUDACION: %s" %dataInmatlab
			if dataInmatlab=="P":
				print "reanudacion de mision correcto"
				break
			time.sleep(0.2)
		'''
		FSM(1)
		sleep(.3)
		FSM(1)
		sleep(.3)
		print "check"
		FSM(1)
		sleep(.3) 
		print "check2"
		FSM(1)
		sleep(.3)
		FSM(1)  
		sleep(.3)
		FSM(1)  
		sleep(.3)
		FSM(0)  
		sleep(2)
		
		
	
if __name__ == '__main__':
    main()
/**
 * <p>Ketai Sensor Library for Android: http://KetaiProject.org</p>
 *
 * <p>Ketai Location Features:
 * <ul>
 * <li>Uses GPS location data (latitude, longitude, altitude (if available)</li>
 * <li>Updates if location changes by 1 meter, or every 10 seconds</li>
 * <li>If unavailable, defaults to system provider (cell tower or WiFi network location)</li>
 * </ul>
 * <p>Syntax:
 * <ul>
 * <li>onLocationEvent(latitude, longitude, altitude)</li>
 * <li>onLocationEvent(latitude, longitude)</li>
 * <li>onLocationEvent(latitude, longitude, altitude)</li>
 * </p>
 * <p>Updated: 2012-10-21 Daniel Sauter/j.duran</p>
 */

import ketai.sensors.*; 
import hypermedia.net.*;

long last_sent, last_recv = 0;
String server_st;
double longitude, latitude, altitude;
KetaiLocation location;
UDP udp;

int xspacing = 2;   // How far apart should each horizontal location be spaced
int w;              // Width of entire wave
int maxwaves = 4;   // total # of waves to add together

float theta = 0.0;
float[] amplitude = new float[maxwaves];   // Height of wave
float[] dx = new float[maxwaves];          // Value for incrementing X, to be calculated as a function of period and xspacing
float[] yvalues = new float[1024];



void mouseReleased() {
  sendMsg("MOU," + mouseX + "," + mouseY);
}

void sendMsg(String msg) {
  String ip       = "224.0.0.1";  // the remote IP address
  int port        = 6100;    // the destination port

  // send the message

  if (udp.send( msg, ip, port )) {
    println( "Sending: \"" + msg +"\" to "+ip+" on port "+port );
    last_sent = millis();
  }
}

void receive( byte[] data ) {
  String str = new String(data);
  String[] parts = str.split(",");


  print(parts[0]);
  if (parts[0].equals("PSD"))
    // Starting at 1
    for (int i = 1; i < parts.length; i++) {

      yvalues[i-1] = Float.parseFloat(parts[i]);
      if ( i == 500) {
        println(parts[i]);

        println(yvalues[500-1]);
      }
    }
  last_recv = millis();
  server_st = "OK";
}

String serverState() {  

  if ( last_sent - last_recv > 1000)
    server_st = "DISCONECTED";

  return server_st;
}

void renderWave() {
  // A simple way to draw the wave with an ellipse at each location
  //noStroke();
  //fill(255, 50);
  //ellipseMode(CENTER);
  fill(255);
  stroke(153);
  rect(100, (height/2) *2/3 , width-200, height-500, 70);

  stroke(204, 102, 0);
  for (int x = 1; x < yvalues.length-1; x++) {
    //ellipse( 200 + x*xspacing, height*2/3 - 200*yvalues[x], 16, 16);
    line(200 + (x-1)*xspacing, height*2/3 - 200*yvalues[x-1],  200 + (x)*xspacing, height*2/3 - 200*yvalues[x]);
  }
}

void setup() {
  fullScreen();
  orientation(LANDSCAPE);  
  //smooth();
  textAlign(CENTER, CENTER);
  textSize(36);

  udp = new UDP( this, 6000, "224.0.0.1" );
  udp.log( true );     // <-- printout the connection activity
  udp.listen( true );
  sendMsg("EVN,STARTING");
}

void draw() {
  if (location == null)
    location = new KetaiLocation(this);

  background(78, 93, 75);

  renderWave();

  if (location.getProvider() == "none")
    text("Location data is unavailable. \n" +
      "Please check your location settings.", 0, 0, width, height/3);
  else
    text("Latitude: " + latitude + "\n" + 
      "Longitude: " + longitude + "\n" + 
      "Altitude: " + altitude + "\n" + 
      "Provider: " + location.getProvider(), 0, 0, width, height/3);  
  // getProvider() returns "gps" if GPS is available
  // otherwise "network" (cell network) or "passive" (WiFi MACID)
  //if (location.getLocation() != null)
  //{
  //   sendMsg("EVN,UPDATING");
  //  latitude = location.getLocation().getLatitude();
  //  longitude = location.getLocation().getLongitude();
  //  altitude = location.getLocation().getAltitude();
  //}
  text("Server: \n" + serverState(), 100, 100, 300, 300);

  delay(30);
  //sendMsg("SCR," + String.valueOf(height) + "," + String.valueOf(width));
  sendMsg("PSD," + String.valueOf(1024));
}

void onLocationEvent(double _latitude, double _longitude, double _altitude)
{
  longitude = _longitude;
  latitude = _latitude;
  altitude = _altitude;
  sendMsg("LOC," + latitude + "," + longitude + "," + altitude);
}

void onDestroy()
{
  sendMsg("EVN,CLOSED");
  super.onDestroy();
}
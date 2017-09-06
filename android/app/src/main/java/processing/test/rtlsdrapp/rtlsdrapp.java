package processing.test.rtlsdrapp;

import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import ketai.sensors.*; 
import hypermedia.net.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class rtlsdrapp extends PApplet {



long last_sent, last_recv = 0;
String server_st;
double longitude, latitude, altitude;
KetaiLocation location;
UDP udp;

int xspacing = 2;   // How far apart should each horizontal location be spaced
float[] yvalues = new float[1024];

public void mouseReleased() {  
  bandwidthAdjust(mouseX, mouseY);

}

public void mouseDragged() {
  bandwidthAdjust(mouseX, mouseY);
}

public void sendMsg(String msg) {
  String ip       = "224.0.0.1";  // the remote IP address
  int port        = 6100;    // the destination port

  // send the message

  if (udp.send( msg, ip, port )) {
    println( "Sending: \"" + msg +"\" to "+ip+" on port "+port );
    last_sent = millis();
  }
}

public void receive( byte[] data ) {
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

public String serverState() {  

  if ( last_sent - last_recv > 1000)
    server_st = "DISCONECTED";

  return server_st;
}


public void setup() {
  
  orientation(LANDSCAPE);  
  //smooth();
  textAlign(CENTER, CENTER);
  textSize(36);

  udp = new UDP( this, 6000, "224.0.0.1" );
  udp.log( true );     // <-- printout the connection activity
  udp.listen( true );
  sendMsg("EVN,STARTING");
}

public void draw() {
  if (location == null)
    location = new KetaiLocation(this);

  background(78, 93, 75);

  renderGraph();

  if (location.getProvider() == "none")
    text("Location data is unavailable. \n" +
      "Please check your location settings.", 0, 0, width, height/3);
  else
    text("Latitude: " + latitude + "\n" + 
      "Longitude: " + longitude + "\n" + 
      "Altitude: " + altitude + "\n" + 
      "Provider: " + location.getProvider(), 0, 0, width, height/3);  

  text("Server: \n" + serverState(), 100, 100, 300, 300);

  delay(30);
  sendMsg("PSD," + String.valueOf(bandwidth));
}

public void onLocationEvent(double _latitude, double _longitude, double _altitude)
{
  longitude = _longitude;
  latitude = _latitude;
  altitude = _altitude;
  sendMsg("LOC," + latitude + "," + longitude + "," + altitude);
}

public void onDestroy()
{
  sendMsg("EVN,CLOSED");
  super.onDestroy();
}

float sample_rate = 1e6f;
float center_freq = 102.8f;
float df = sample_rate / yvalues.length;
float bandwidth = 150e3f;

int center_freq_x = 200 + yvalues.length/2 * xspacing;
int left_marker_x = PApplet.parseInt(PApplet.parseFloat(center_freq_x) - bandwidth/(2*df));
int right_marker_x = PApplet.parseInt(PApplet.parseFloat(center_freq_x) + bandwidth/(2*df));

public void bandwidthAdjust(int x, int y) {
  if ( y > height*2/3 || y < height*2/3 - 300) return;

  if (x > center_freq_x)
    bandwidth = (x - center_freq_x) * df *xspacing/2;
  else 
  bandwidth = (center_freq_x - x)* df *xspacing/2;

  if (bandwidth > sample_rate) bandwidth = sample_rate;
}

public void renderGraph() {

  fill(255);
  stroke(153);
  rect(100, (height/2.0f) *2/3, (width-200), (height-500), 70);

  line(200, height*2/3 + 200, 200, height*2/3 - 300); // Y axis
  line(200, height*2/3 + 200, 200 + yvalues.length*xspacing, height*2/3 + 200); // X axis
  line(200 + yvalues.length/2*xspacing, height*2/3 +200, 200 + yvalues.length/2*xspacing, height*2/3+220); // X axis


  stroke(204, 102, 0);
  for (int x = 1; x < yvalues.length-1; x++) {
    line(200 + (x-1)*xspacing, height*2/3 - 200*yvalues[x-1], 200 + (x)*xspacing, height*2/3 - 200*yvalues[x]);
  }

  fill(255, 0, 0, 128);  //color red semi-transparent
  stroke(128, 0, 255, 128); //color purple semi-transparent

  left_marker_x = PApplet.parseInt(PApplet.parseFloat(center_freq_x) - xspacing*bandwidth/(2*df));
  right_marker_x = PApplet.parseInt(PApplet.parseFloat(center_freq_x) + xspacing*bandwidth/(2*df));
  rect( left_marker_x, (height) *2/3 + 200, right_marker_x - left_marker_x, -300);


  text("BW : " + str(bandwidth/1000) + " kHz", 200 + yvalues.length/2*xspacing, height*2/3 - 125);


  fill(0);
  text( str(center_freq - sample_rate/(1e6f*2)) + " MHz", 200, height*2/3 + 250);
  text(str(center_freq) + " MHz", center_freq_x, height*2/3 + 250);
  text(str(center_freq + sample_rate/(1e6f*2)) + " MHz", 200 + yvalues.length*xspacing, height*2/3 + 250);
}
  public void settings() {  fullScreen(); }
}

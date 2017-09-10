import hypermedia.net.*;

UDP udp;
long last_sent, last_recv = 0;
String server_st = "DISCONECTED";

void initComm() {
  udp = new UDP( this, 6000, "224.0.0.1" );
  udp.log( true );     // <-- printout the connection activity
  udp.listen( true );
  sendMsg("EVN,STARTING");
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
  if (parts[0].equals("PSD")) {
    // Starting at 1
    for (int i = 1; i < parts.length; i++) {

      yvalues[i-1] = Float.parseFloat(parts[i]);
      if ( i == 500) {
        println(parts[i]);

        println(yvalues[500-1]);
      }
    }
  } else if (parts[0].equals("POW")) {
    estimated = parts[1];
  } else if (parts[0].equals("SRT")) {
    axisX.setRange( Float.parseFloat(parts[1]) );
    sample_rate = Float.parseFloat(parts[1]);
  } else if (parts[0].equals("CFQ")) {
    axisX.setCenter(Float.parseFloat(parts[1]) );
  }

  last_recv = millis();
  server_st = "OK";
}

String serverState() {  

  if ( last_sent - last_recv > 1000)
    server_st = "DISCONECTED";

  return server_st;
}
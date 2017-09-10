
private static final int NFFT = 1024;
float[] yvalues = new float[NFFT];

int bw_l;
int bw_r;
float bw;
float sample_rate = 2.0;
String estimated = "0.0 dB";

String activeMarker = "none";

private void renderGraph() {

  stroke(#2B59A5);
  strokeWeight(3);
  for (int x = 1; x < yvalues.length - 1; x++) {
    line(axisX.getX1() + (x - 1) * axisX.getSpacing(), axisY.getY1() - axisY.getLength() * yvalues[x - 1], 
      axisX.getX1() + + (x) * axisX.getSpacing(), axisY.getY1() - axisY.getLength() * yvalues[x]);
  }
}

private void renderBW() {
  fill(#71B4E5, 128);  
  stroke(255, 255, 128, 0);

  if (mousePressed) {
    if ( mouseY - axisY.getY1() < axisY.getLength() && mouseY > axisY.getY1()    
      && mouseX - axisX.getX1() < axisX.getLength() && mouseX > axisX.getX1() ) {
      if (mouseX < leftMarker) {
        leftMarker = mouseX;
      } else if (mouseX > rightMarker) {
        rightMarker = mouseX;
      } else {
        if (activeMarker == "left") {
          leftMarker = mouseX;
        } else if (activeMarker == "right") {
          rightMarker = mouseX;
        }

        if (abs(mouseX - rightMarker) < width/100) activeMarker = "right";
        if (abs(mouseX - leftMarker) < width/100) activeMarker = "left";
      }

      bw_l = int( (leftMarker- axisX.getX1()) /axisX.getSpacing()) ;
      sendMsg("BWL,"+ str(bw_l));  
      bw_r = int( (rightMarker- axisX.getX1()) /axisX.getSpacing()) ;
      sendMsg("BWR,"+ str(bw_r));
    }
  } else activeMarker = "none";

  int dX = abs(rightMarker - leftMarker);  
  rect( leftMarker, axisY.getY2(), dX, axisY.getY1() - axisY.getY2());

  fill(0);

  bw = (sample_rate*1e3)*(bw_r - bw_l)*1.0/NFFT;
  bwr.setText("Selected BW : " + str( int(10*bw)/10.0) + " KHz\n" + "Estimated Power : " + estimated );
}
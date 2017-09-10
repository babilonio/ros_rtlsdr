
private static final int NFFT = 1024;
float[] yvalues = new float[NFFT];

int bw_l;
int bw_r;
float bw;
float sample_rate = 2.4e6;
String estimated = "0.0 dB";

private void renderGraph() {

  stroke(#2B59A5);
  strokeWeight(3);
  for (int x = 1; x < yvalues.length - 1; x++) {
    line(axisX.getX1() + (x - 1) * axisX.getSpacing(), axisY.getY1() - 480 * yvalues[x - 1], 
      axisX.getX1() + + (x) * axisX.getSpacing(), axisY.getY1() - 480 * yvalues[x]);
  }
}

private void renderBW() {
  fill(#71B4E5, 128);  
  stroke(255, 255, 128, 0);

  if (mousePressed) {
    if ( mouseY - axisY.getY1() < axisY.getLength() && mouseY > axisY.getY1() ) {   
      if ( mouseX - axisX.getX1() < axisX.getLength()/2 && mouseX > axisX.getX1() ) {
        leftMarker = mouseX;
        bw_l = int((axisX.getCenter() - leftMarker)/axisX.getSpacing());
        sendMsg("BWL,"+ str(bw_l));
      } else if ( mouseX - axisX.getX1() < axisX.getLength() && mouseX > axisX.getX1() ) {
        rightMarker = mouseX;   
        bw_r = int((rightMarker - axisX.getCenter())/axisX.getSpacing());
        sendMsg("BWR,"+ str(bw_r));
      }
    }
  }
  int dX = abs(rightMarker - leftMarker);  
  rect( leftMarker, axisY.getY2(), dX, axisY.getY1() - axisY.getY2());

  fill(0);

  bw = (sample_rate/1e3)*(bw_l + bw_r)*1.0/NFFT;
  bwr.setText("Selected BW : " + str( int(10*bw)/10.0) + " KHz\n" + "Estimated Power : " + estimated );
}
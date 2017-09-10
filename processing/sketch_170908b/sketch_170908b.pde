
Rectangle buttonCenterFreq, buttonSampleRate, activeButton;
Rectangle r, bwr, infor;
CheckBox checkBoxMap;
Axis axisY, axisX;
PImage img;

int leftMarker = 0;
int rightMarker = 0;


public void setup() {

  orientation(LANDSCAPE);
  smooth();
  textAlign(CENTER, CENTER);
  println(width, height);
  img = loadImage("Logo_UMA.png");

  r = new Rectangle( width/200, height/ 3, (width - width/100), height*3/5, height/10);

  buttonCenterFreq = new Rectangle( r.getX() + round(r.getSX()*4/ 6.0), 
    r.getY() -  + round(r.getSY()*8/ 30.0), 
    r.getSX()/7, r.getSY()/10, 1);   
  buttonCenterFreq.setTextLabel("SET CenterFreq", 255);
  buttonCenterFreq.setCmd("SCF");

  buttonSampleRate = new Rectangle( r.getX() + round(r.getSX()*4/ 6.0), 
    r.getY() -  + round(r.getSY()*4/ 30.0), 
    r.getSX()/7, r.getSY()/10, 1);   
  buttonSampleRate.setTextLabel("SET SampleRate", 255);
  buttonSampleRate.setCmd("SSR");

  checkBoxMap = new CheckBox( buttonCenterFreq.getX() + buttonCenterFreq.getSX()*5/4, 
    buttonCenterFreq.getY(), buttonCenterFreq.getSX(), buttonCenterFreq.getSY(), 1);
  checkBoxMap.setTextLabel("Mapping ", 255);

  bwr = new Rectangle(r.getX() + round(r.getSX()*2/ 6.0), 
    r.getY() -  + round(r.getSY()*7.5/ 30.0), 
    r.getSX()/4, r.getSY()/5, r.getR()/10);  
  bwr.setTextLabel("Selected BW : ", 255);

  infor = new Rectangle(r.getX() + round(r.getSX()*1/ 10.0), 
    r.getY() -  + round(r.getSY()*7.5/ 30.0), 
    r.getSX()/6, r.getSY()/5, r.getR()/10);  
  infor.setTextLabel("SERVER: OK\nGPS: OK", color(#67FAB2));

  int x = r.getX() + round(r.getSX()/ 20.0);
  int y = r.getY() + round(r.getSY()/ 20.0);
  int dy = r.getSY() - round(r.getSY()/5.0);
  int dx = r.getSX() - round(r.getSX()/10.0);

  axisY = new Axis(x, y, x, y + dy, 5, true);
  axisX = new Axis(x, y + dy, x + dx, y + dy, 4, false);
  axisX.setSpacing( (axisX.getX2() - axisX.getX1())/(1.0*NFFT) );

  axisY.setCenter(-25);
  axisY.setRange(50);
  axisX.setCenter(102.8);
  axisX.setRange(2.4);

  leftMarker = axisX.getCenter();
  rightMarker = axisX.getCenter();

  initComm();
}

public void draw() {

  if (location == null)
    location = new KetaiLocation(this);

  background(#E1E1E5);
  stroke(0);
  strokeWeight(2);

  textSize(height/40);

  fill(255);
  r.display();
  fill(#2B59A5);
  buttonCenterFreq.display();
  fill(#2B59A5);
  buttonSampleRate.display();  
  fill(#2B59A5);
  checkBoxMap.display();
  fill(#71B4E5); 
  stroke(255);
  bwr.display();
  fill(#71B4E5);

  if (server_st.equals("OK")) {
    stroke(#67FAB2);
    infor.setTextLabel("SERVER: OK\nGPS: OK", color(#67FAB2));
  } else {
    stroke(#FA544E);
    infor.setTextLabel("SERVER: "+ server_st, color(#FA544E));
  }
  infor.display();

  textSize(height/50);
  fill(0);
  stroke(0);
  axisY.display();
  axisX.display();

  renderGraph();
  renderBW();

  fill(0);
  text("[dB/MHz]", r.getX() + r.getSX()/2, r.getY() + round(r.getSY()*14/15.0) );
  image(img, r.getX(), 0, r.getSY()/ 4, r.getSY()/4);
}


public void settings() {
  fullScreen();
}
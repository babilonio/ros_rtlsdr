

public class Rectangle {
  private int x, y, sx, sy, r;
  private String textLabel, text;
  private int colorText;

  public Rectangle(int x, int y, int sx, int sy, int r) {
    this.x = x;
    this.y = y;
    this.sx = sx;
    this.sy = sy;
    this.r = r;
  }

  public int getX() { 
    return x;
  }
  public int getY() { 
    return y;
  }
  public int getSX() { 
    return sx;
  }
  public int getSY() { 
    return sy;
  }
  public int getR() { 
    return r;
  }

  public void setTextLabel(String text, int colorText) {
    this.colorText = colorText;
    this.textLabel = text;
    this.text = text;
  }

  public String getText() {
    return text;
  }

  public void setText(String text) {
    if (text.equals("label"))
      this.text = textLabel;
    else
      this.text = text;
  }

  public void display() {
    rect(x, y, sx, sy, r);
    if (text != null) {
      fill(colorText);
      text(text, x + sx/2, y + sy/2);
    }
  }
}

public class Axis {
  private int x1, y1, x2, y2, divisions;
  private boolean vertical;
  private float center, range, spacing;

  public Axis(int x1, int y1, int x2, int y2, int divisions, boolean vertical) {
    this.x1 = x1;
    this.y1 = y1;
    this.x2 = x2;
    this.y2 = y2;
    this.divisions = divisions;
    this.vertical = vertical;
  }

  public void setRange(float range) {
    this.range = range;
  }

  public void setCenter(float center) {
    this.center = center;
  }
  public void setSpacing(float spacing) {
    this.spacing = spacing;
  }

  public float getSpacing() {
    return this.spacing;
  }

  public float getLength() {
    int l;
    if (vertical)
      l = abs(y2 - y1);
    else 
    l = abs(x2 - x1);
    return l;
  }

  public int getCenter() {
    int center;
    if (vertical)
      center = round(getLength()/2.0 + getY1());
    else 
    center = round(getLength()/2.0 + getX1());
    return center;
  }

  public int getX1() {
    return x1;
  }

  public int getX2() {
    return x2;
  }
  public int getY1() {
    return y1;
  }
  public int getY2() {
    return y2;
  }

  public void display() {
    line(x1, y1, x2, y2);

    if (divisions > 0) {
      float dval = range / (1.0*divisions);

      if (vertical) {
        float start = center + range/2.0;
        float dy = abs(y2 - y1)/(1.0*divisions) ;
        for (int i = 0; i <= divisions; i++) {
          line(x1, y1 + dy*i, x1 - width/200, y1 + dy*i);
          text(str( start - int(10*dval*i)/10), x1 - width/50, y1 + dy*i);
        }
      } else {
        float start = center - range/2.0;
        float dx = abs(x2 - x1)/(1.0*divisions) ;
        for (int i = 0; i <= divisions; i++) {
          line(x1 + dx*i, y1, x1  + dx*i, y1 + height/100);
          text(str( int(10*(start + dval*i))/10.0), x1  + dx*i, y1 + height/50);
        }
      }
    }
  }
}

public void mousePressed() {
  if (mouseX - buttonCenterFreq.getX() < buttonCenterFreq.getSX() && mouseX > buttonCenterFreq.getX()
    && mouseY - buttonCenterFreq.getY() < buttonCenterFreq.getSY()  && mouseY > buttonCenterFreq.getY() && activeButton == null) {
    activeButton = buttonCenterFreq;
  } else if (mouseX - buttonSampleRate.getX() < buttonSampleRate.getSX() && mouseX > buttonSampleRate.getX()
    && mouseY - buttonSampleRate.getY() < buttonSampleRate.getSY()  && mouseY > buttonSampleRate.getY() && activeButton == null) {
    activeButton = buttonSampleRate;
  } else {
    if (activeButton != null) {
      activeButton.setText("label");
      activeButton = null;
      //hideVirtualKeyboard();
    }
  }

  if (activeButton != null) {
    activeButton.setText("");
    //showVirtualKeyboard();
  }
}
public void keyPressed()
{
  if (activeButton != null) {
    if (keyCode != 66) {
      String s = activeButton.getText();
      if (keyCode == 63)
        activeButton.setText(s.substring(0, s.length() - 1));
      else 
      activeButton.setText(s + key);
    } else {
      activeButton.setText("label");
      activeButton = null;
      //hideVirtualKeyboard();
    }
  }
}

//public void showVirtualKeyboard()
//{
//    InputMethodManager imm = (InputMethodManager) getActivity().getSystemService(Context.INPUT_METHOD_SERVICE);
//    imm.toggleSoftInput(InputMethodManager.SHOW_FORCED, 0);

//}
//public void hideVirtualKeyboard()
//{
//    InputMethodManager imm = (InputMethodManager) getActivity().getSystemService(Context.INPUT_METHOD_SERVICE);
//    imm.toggleSoftInput(InputMethodManager.HIDE_IMPLICIT_ONLY, 0);
//}

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
      } else if ( mouseX - axisX.getX1() < axisX.getLength() && mouseX > axisX.getX1() ) {
        rightMarker = mouseX;
      }
    }
  }
  int dX = abs(rightMarker - leftMarker);  
  rect( leftMarker, axisY.getY2(), dX, axisY.getY1() - axisY.getY2());

  fill(0);

  int bw_l = int((axisX.getCenter() - leftMarker)/axisX.getSpacing());
  int bw_r = int((rightMarker - axisX.getCenter())/axisX.getSpacing());
  float bw = 2.4e3*(bw_l + bw_r)*1.0/NFFT;

  bwr.setText("Selected BW : " + str( int(10*bw)/10.0) + " KHz\n" + "Estimated Power : -10.0 dB");
}


private static final int NFFT = 1024;
float[] yvalues = new float[NFFT];

Rectangle buttonCenterFreq, buttonSampleRate, activeButton;
Rectangle r, bwr;
Axis axisY, axisX;
PImage img;

int leftMarker = 0;
int rightMarker = 0;

public void setup() {

  orientation(LANDSCAPE);
  smooth();
  textAlign(CENTER, CENTER);
  

  println(width, height);

  r = new Rectangle( width/200, height/ 3, (width - width/100), height*3/5, height/10);

  buttonCenterFreq = new Rectangle( r.getX() + round(r.getSX()*4/ 6.0), 
    r.getY() -  + round(r.getSY()*8/ 30.0), 
    r.getSX()/7, r.getSY()/10, r.getR()/10);   
  buttonCenterFreq.setTextLabel("Center Freq [MHz]", 255);

  buttonSampleRate = new Rectangle( r.getX() + round(r.getSX()*4/ 6.0), 
    r.getY() -  + round(r.getSY()*4/ 30.0), 
    r.getSX()/7, r.getSY()/10, r.getR()/10);   
  buttonSampleRate.setTextLabel("Sample Rate [MHz]", 255);

  bwr = new Rectangle(r.getX() + round(r.getSX()*2/ 6.0), 
    r.getY() -  + round(r.getSY()*7/ 30.0), 
    r.getSX()/4, r.getSY()/5, r.getR()/10);  
  bwr.setTextLabel("Selected BW : ", 255);

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

  img = loadImage("Logo_UMA.png");

  leftMarker = axisX.getCenter();
  rightMarker = axisX.getCenter();
}

public void draw() {

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
  fill(#71B4E5); 
  bwr.display();
  
  textSize(height/50);
  fill(0);
  axisY.display();
  axisX.display();
  
  renderGraph();
  renderBW();

  fill(0);
  text("[dB/MHz]", r.getX() + r.getSX()/2, r.getY() + round(r.getSY()*14/15.0) );
  image(img, r.getX(), 0, r.getSY()/ 4, r.getSY()/4);


  //  text("Server: \n" + "OK", 100, 100, 300, 300);
  //  text("Power: \n" + str(-48.0), width - 300, 100, 300, 300);
}


public void settings() {
  fullScreen();
}
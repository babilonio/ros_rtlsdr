

public class Rectangle {
  protected int x, y, sx, sy, r;
  private String textLabel, text, cmd;
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


  public void setCmd(String cmd) {
    this.cmd = cmd;
  }
  public String getCmd() {
    return cmd;
  }

  public void display() {
    rect(x, y, sx, sy, r);
    if (text != null) {
      fill(colorText);
      text(text, x + sx/2, y + sy/2);
    }
  }
}

public class CheckBox extends Rectangle {
  int boxX, boxY;
  boolean checked;

  public CheckBox(int x, int y, int sx, int sy, int r) {
    super(x, y, sx, sy, r);
    boxX = x + sx*5/6;
    boxY = y + sy*2/5;
    checked = false;
  }

  @Override
    public void display() {
    super.display();
    fill(255);
    rect(boxX, boxY, this.sx/15, this.sy/4);

    if (checked) {
      line(boxX, boxY, boxX+this.sx/15, boxY+this.sy/4);
      line(boxX, boxY+this.sy/4, boxX+this.sx/15, boxY);
    }
  }


  void check() { 
    checked = true;
  }
  void uncheck() { 
    checked = false;
  }
  void toggle() { 
    checked = !checked;
  }
  boolean isChecked() { 
    return checked;
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
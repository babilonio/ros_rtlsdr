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
      hideVirtualKeyboard();
    }
  }

  if (mouseX - checkBoxMap.getX() < checkBoxMap.getSX() && mouseX > checkBoxMap.getX()
    && mouseY - checkBoxMap.getY() < checkBoxMap.getSY()  && mouseY > checkBoxMap.getY()) {
    checkBoxMap.toggle();
    sendMsg("MAP,"+str(int(checkBoxMap.isChecked())));
  } 

  if (activeButton != null) {
    activeButton.setText("");
    showVirtualKeyboard();
  }
}

public void keyPressed()
{
  if (activeButton != null) {
    if (keyCode != 66 && keyCode != 10) {
      String s = activeButton.getText();     
      if (keyCode == 63)
        activeButton.setText(s.substring(0, s.length() - 1));
      else 
      activeButton.setText(s + key);
    } else {
      String s = activeButton.getText();

      try {
        float val = Float.parseFloat(s);
        println(val);
        if (activeButton.getCmd()!=null)
          sendMsg(activeButton.getCmd() + "," + str(val));
      }
      catch(NumberFormatException e) {
        println("Error parsing float");
      }

      activeButton.setText("label");
      activeButton = null;
      hideVirtualKeyboard();
    }
  }
}
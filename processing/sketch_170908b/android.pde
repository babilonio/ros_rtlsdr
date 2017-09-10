
//import ketai.sensors.*; 
double longitude, latitude, altitude;

public void showVirtualKeyboard()
{
//    InputMethodManager imm = (InputMethodManager) getActivity().getSystemService(Context.INPUT_METHOD_SERVICE);
//    imm.toggleSoftInput(InputMethodManager.SHOW_FORCED, 0);

}
public void hideVirtualKeyboard()
{
//    InputMethodManager imm = (InputMethodManager) getActivity().getSystemService(Context.INPUT_METHOD_SERVICE);
//    imm.toggleSoftInput(InputMethodManager.HIDE_IMPLICIT_ONLY, 0);
}

void onLocationEvent(double _latitude, double _longitude, double _altitude)
{
  longitude = _longitude;
  latitude = _latitude;
  altitude = _altitude;
  sendMsg("LOC," + latitude + "," + longitude + "," + altitude);
}
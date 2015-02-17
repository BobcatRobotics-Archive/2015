package org.usfirst.frc.team177.lib;

import java.io.IOException;
import java.util.Vector;

//Adapted/stolen for ChezzyPoofs 2014 code

/**
 * Manages constant values used everywhere in the robot code.
 *
 * 
 */
public abstract class ConstantsBase {
  private static final Vector<Constant> constants = new Vector<Constant>();
  private static final String CONSTANTS_FILE_PATH = "Constants.txt";

   /**
   * Reads the constants file and overrides the values in this class for any constants it contains.
   */
  public static void readConstantsFromFile() {
    try {
      String file = BobcatUtils.getFile(CONSTANTS_FILE_PATH);
      if (file.length() < 1) {
        throw new IOException("Not over riding constants");
      }
      // Extract each line separately.
      String[] lines = BobcatUtils.split(file, "\n");
      for (int i = 0; i < lines.length; i++) {
        // Extract the key and value.
        String[] line = BobcatUtils.split(lines[i], "=");
        if (line.length != 2) {
          System.out.println("Error: invalid constants file line: " +
                            (lines[i].length() == 0 ? "(empty line)" : lines[i]));
          continue;
        }

        boolean found = false;
        // Search through the constants until we find one with the same name.
        for (int j = 0; j < constants.size(); j++) {
          Constant constant = (Constant)constants.elementAt(j);
          if (constant.getName().compareTo(line[0]) == 0) {
            System.out.println("Setting " + constant.getName() + " to " + Double.parseDouble(line[1]));
            constant.setVal(Double.parseDouble(line[1]));
            found = true;
            break;
          }
        }

        if (!found)
          System.out.println("Error: the constant doesn't exist: " + lines[i]);
      } 
    } catch(IOException e ) {
      System.out.println(e);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
  
  public static String generateHtml() {
    String str = "<html><head><title>Chezy Constants</title></head><body>"
            + "<form method=\"post\">";
    str += "<table>";
    for(int i = 0; i < constants.size(); ++i) {
      str+= ((Constant) constants.elementAt(i)).toHtml();
    }
    str += "</table><input type=\"submit\" value=\"Submit\">";
    str += "</form>";
    str +=  "</body></html>";
    return str;
  }
  
  public static void writeConstant(String name, double value) {
    Constant constant;
    for (int i = 0; i < constants.size(); i++) {
      constant = ((Constant) constants.elementAt(i));
      if (constant.name.equals(name)) {
        constant.setVal(value);
      }
    }
  }


 /**
  * Handles an individual value used in the Constants class.
  */
  public static class Constant {
    private String name;
    private double value;

    public Constant(String name, double value) {
      this.name = name;
      this.value = value;
      constants.addElement(this);
    }

    public String getName(){
      return name;
    }

    public double getDouble() {
      return value;
    }

    public int getInt() { 
      return (int)value;  //DAS storing ints as doubles is asking for trouble, need to rework this somehow
    }

    public void setVal(double value){
      this.value = value;
    }
    
    public String toHtml() {
      String str = "<html>" +
              "<tr><td align=right>" + this.name +":</td><td>"
              + "<input type='text' value=\""+this.value+"\" name=\"" + this.name
              + "\"> </td></tr>";
      
      return str;
    }

    public String toString(){
      return name + ": " + value;
    }
  }
}

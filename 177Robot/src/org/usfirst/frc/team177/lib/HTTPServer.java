package org.usfirst.frc.team177.lib;

//Adapted/stolen from ChezzyPoofs 2014 code

import org.usfirst.frc.team177.robot.Constants;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Vector;

public class HTTPServer implements Runnable {

  private static int PORT = 10177;
  private Vector<Socket> connections;

  public void pushData(String message) {
    for (int i = 0; i < connections.size(); ++i) {
      try {
        OutputStream os = (connections.elementAt(i)).getOutputStream();
        os.write(message.getBytes());
        os.close();
      } catch (IOException e) {
        connections.removeElementAt(i);
        System.out.println(e);
      }
    }
  }

  /* 
   * Spawns a new thread to handle requests and responses 
   */
  private class ConnectionHandler implements Runnable {

    Socket connection;

    public ConnectionHandler(Socket s) {
      connection = s;
    }

    public void run() {
      try {
    	BufferedReader in = new BufferedReader(new InputStreamReader(connection.getInputStream()));
    	
        String x = in.readLine();
        String req = x +'\n';
        
        while (in.ready()) {
          x = in.readLine();
          req += x +'\n';
        }
        OutputStream os = connection.getOutputStream();

        route(req, os);

        os.close();
        in.close();
        connection.close();

      } catch (IOException e) {
      }
    }
  }

  public void run() {
    ServerSocket s = null;
    try {
      s = new ServerSocket(PORT);
      while (true) {
    	Socket remote = s.accept();  
        Thread t = new Thread(new ConnectionHandler(remote));
        t.start();
        connections.addElement(remote);
      }
    } catch (IOException e) {
      System.out.println("There was an error " + e);
    }
  }

  public HTTPServer() {
    PORT = 10177;
    connections = new Vector<Socket>();
  }

  public HTTPServer(int port) {
    PORT = port;
    connections = new Vector<Socket>();
  }

  public static void route(String req, OutputStream os) {

    int end = req.indexOf('\n');
    end = end > 0 ? end : req.length();
    
    String header = req.substring(0, end);
    String[] reqParams = BobcatUtils.split(header, " ");
    if (reqParams.length < 2) {
      return;
    }
    String type = reqParams[0];
    String path = reqParams[1];
    try {
      if (type.equals("GET")) {
        if (path.startsWith("/constants")) {
          HtmlResponse res = new HtmlResponse(Constants.generateHtml());
          os.write(res.toString().getBytes());
        } 
        else if (path.equals("/")) {
          HtmlResponse res = new HtmlResponse(BobcatUtils.getFile("/www/index.html"));
          os.write(res.toString().getBytes());
        } else {
          // Returns a file
          HtmlResponse res = new HtmlResponse(BobcatUtils.getFile("/www" + path));
          os.write(res.toString().getBytes());
          //os.write(HtmlResponse.PAGE_NOT_FOUND.getBytes());
        }
      } else if (type.equals("POST")) {
        if (path.startsWith("/constants")) {
          Hashtable<String, Double> args = parsePost(req);
          for (Enumeration<String> en = args.keys(); en.hasMoreElements();) {
            String key = (String) en.nextElement();
            Double val = (Double) args.get(key);
            ConstantsBase.writeConstant(key, val.doubleValue());
          }
          HtmlResponse res = new HtmlResponse("<html><head><meta http-equiv=\"refresh\" content=\"0; url=http://roboRio-177.local:" + PORT + "/constants\"></head><body>Redirecting</body></html>");
          os.write(res.toString().getBytes());
        }
      } else {  
    	HtmlResponse res = new HtmlResponse(HtmlResponse.ERROR);
      	os.write(res.toString().getBytes());
      }
    } catch (IOException e) {
      System.out.println("Exception was caught: " + e.toString());
    }
  }

  /**
   * Parse the whole HTTP request string into a hashtable of the POST args
   */
  private static Hashtable<String, Double> parsePost(String req) {
    Hashtable<String, Double> postArgs = new Hashtable<String, Double>();
    String[] reqLines = BobcatUtils.split(req, "\n");
    String postLine = null;
    for (int i = 0; i < reqLines.length-1; i++) {
      if (reqLines[i].trim().equals("")) {
        postLine = reqLines[i+1];
        break;
      }
    }
    String[] args = BobcatUtils.split(postLine, "&");
    String firstKey = postLine.substring(0, postLine.indexOf("="));
    double firstVal = Double.parseDouble(postLine.substring(postLine.indexOf("=")+1, postLine.indexOf("&")));
    postArgs.put(firstKey, new Double(firstVal));
    for (int i = 0; i < args.length; i++) {
      String[] arg = BobcatUtils.split(args[i], "=");
      try {
        postArgs.put(arg[0], new Double(Double.parseDouble(arg[1])));
      } catch (NumberFormatException e) {
        System.out.println("Cast exception for: " + arg[0] + ", " + arg[1]);
      }
    }
    return postArgs;
  }

}

import hypermedia.net.*;
import toxi.geom.*;
import toxi.processing.*;
/*--------------------------------------------------------------------------------
 按S键Start
 按Q键Quit
 ----------------------------------更改以下数据----------------------------------------------*/
int MPUNumber = 2;//MPU6050芯片数量
//--------------------------------------------------------------------------------
UDP udp; 
int num = 0;
char[] teapotPacket = new char[16];  
int serialCount = 0; 
int synced = 0;
int interval = 0;
int[] teapot = new int[9];
float[] q = new float[4];
int[] countMPU = new int[6];
float[] gravity = new float[3];
float[] euler = new float[3];
float[] ypr = new float[3];
static String FileName = str(day()) + "-" + str(hour()) + "-" + str(minute())+"-"+str(second());
Table table = new Table();
Table angleTable = new Table();
PrintWriter output;
TableRow newRow;
int filled=0;
float [][] qSave = new float[MPUNumber][4];
float [][] eulerSave = new float[MPUNumber][3];
String[][] title = new String[MPUNumber][4];
String[][] angleTitle = new String[MPUNumber][3];
Quaternion[] quat = new Quaternion[MPUNumber];
//--------------------------------------------------------------------------------

public interface Debug { 
  public final boolean ENABLE = false;
}  
public interface Data2CsvFile { 
  public final boolean ENABLE = true;
}  
public interface Angle2CsvFile { 
  public final boolean ENABLE = true;
}  

//--------------------------------------------------------------------------------

void setup() {
  udp = new UDP( this, 1234 ); 
  //udp.log( true );        
  udp.listen( true );
  for (int n = 0; n<MPUNumber; n++) {
    
    
    quat[n] = new Quaternion(1, 0, 0, 0);
    
    
    for (int m = 0; m<4; m++){
      StringBuffer sbu = new StringBuffer();
      sbu.append((char)(65+n));
      String titleString = sbu.toString()+String.valueOf(m);
      title[n][m]=titleString;
      if (Data2CsvFile.ENABLE)    
      { 
        table.addColumn(titleString);
      }
    }
    
    for (int m = 0; m<3; m++){
      StringBuffer sbu = new StringBuffer();
      sbu.append((char)(65+n));
      String titleString = sbu.toString()+String.valueOf(m);
      angleTitle[n][m]=titleString;
      if (Angle2CsvFile.ENABLE)    
      { 
        angleTable.addColumn(titleString);
      }
    }
  }

}
//--------------------------------------------------------------------------------

void draw()
{
}
//--------------------------------------------------------------------------------

void keyPressed() {
  if (key ==  's' || key ==  'S') {
    String ip       = "192.168.4.255"; 
    int port        = 8266;       
    String msg = "" +  key;
    udp.send(msg, ip, port );    
    num = 0;
  } else if (key ==  'q' || key ==  'Q') {
    if (Data2CsvFile.ENABLE)    
    {
      saveTable(table, "CSVData/"+FileName+".csv", "csv");
    }
    if (Angle2CsvFile.ENABLE)    
    {
      saveTable(angleTable, "AngleCSVData/"+FileName+".csv", "csv");
    }
    exit();
  }
}
//--------------------------------------------------------------------------------

void receive( byte[] data ) { 
  interval = millis();
  int kk[] = int(data);
  if (kk.length == 15 && kk[0]=='$') {
    num++;
    //println("start" + int(kk[2]));
    for (int i=0; i < 15; i++) {
      int ch = (kk[i]);
      if (synced == 0 && ch != '$') return;   // initial synchronization - also used to resync/realign if needed
      synced = 1;
      //print (int(ch));

      if ((serialCount == 1 && ch != 2)
        || (serialCount == 13 && ch != '\r')
        || (serialCount == 14 && ch != '\n')) {
        serialCount = 0;
        synced = 0;
        return;
      }

      if (serialCount > 0 || ch == '$') {
        teapotPacket[serialCount++] = (char) ch;
        //println("serialCount" + serialCount);
        if (serialCount == 15) {
          serialCount = 0; 
          q[0] = ((teapotPacket[2] << 8) | teapotPacket[3]) / 16384.0f;
          q[1] = ((teapotPacket[4] << 8) | teapotPacket[5]) / 16384.0f;
          q[2] = ((teapotPacket[6] << 8) | teapotPacket[7]) / 16384.0f;
          q[3] = ((teapotPacket[8] << 8) | teapotPacket[9]) / 16384.0f;
          for (int j = 0; i < 4; i++) if (q[j] >= 2) q[j] = -4 + q[j];

          //println(float(Math.round((q[0])*100))/100+"\t"
          //+ float(Math.round((q[1])*100))/100+"\t"
          //+ float(Math.round((q[2])*100))/100+"\t"
          //+ float(Math.round((q[3])*100))/100);
          gravity[0] = 2 * (q[1]*q[3] - q[0]*q[2]);
                gravity[1] = 2 * (q[0]*q[1] + q[2]*q[3]);
                gravity[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

          euler[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
          euler[1] = -asin(2*q[1]*q[3] + 2*q[0]*q[2]);
          euler[2] = atan2(2*q[2]*q[3] - 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1);


          //println(q[0]+"\t"+q[1]+"\t"+q[2]+"\t"+q[3]);

          //println("euler:\t" + euler[0]*180.0f/PI + "\t" + euler[1]*180.0f/PI + "\t" + euler[2]*180.0f/PI);

                ypr[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
                ypr[1] = atan(gravity[0] / sqrt(gravity[1]*gravity[1] + gravity[2]*gravity[2]));
                ypr[2] = atan(gravity[1] / sqrt(gravity[0]*gravity[0] + gravity[2]*gravity[2]));
    
                // output various components for debugging
                println("q:\t" + round(q[0]*100.0f)/100.0f + "\t" + round(q[1]*100.0f)/100.0f + "\t" + round(q[2]*100.0f)/100.0f + "\t" + round(q[3]*100.0f)/100.0f);
                println("euler:\t" + euler[0]*180.0f/PI + "\t" + euler[1]*180.0f/PI + "\t" + euler[2]*180.0f/PI);
                println("ypr:\t" + ypr[0]*180.0f/PI + "\t" + ypr[1]*180.0f/PI + "\t" + ypr[2]*180.0f/PI);
                
          countMPU[teapotPacket[12]] ++;
          //println("\tMPU-" + int(teapotPacket[12]) 
          //+ "\tnum" + num + "\tmillis: "+ millis() +"\tMPU0: " +countMPU[0]+"\tMPU1: " +countMPU[1]
          //+"\tMPU2: " +countMPU[2]+"\tMPU3: " +countMPU[3]+"\tMPU4: " +countMPU[4]+"\tMPU5: " +countMPU[5]
          //+"\t0xFF-"+ byte(teapotPacket[11]));


          //if ( int(teapotPacket[12]) == 5) {
            
          //print("euler:\t" + euler[0]*180.0f/PI + "\t" + euler[1]*180.0f/PI + "\t" + euler[2]*180.0f/PI);
          
          //print(float(Math.round((q[0])*1000))/1000+"\t"
          //  + float(Math.round((q[1])*1000))/1000+"\t"
          //  + float(Math.round((q[2])*1000))/1000+"\t"
          //  + float(Math.round((q[3])*1000))/1000);
          
          //print(float(Math.round((q[0]*180/PI)*1000))/1000+"\t"
          //  + float(Math.round((q[1]*180/PI)*1000))/1000+"\t"
          //  + float(Math.round((q[2]*180/PI)*1000))/1000+"\t"
          //  + float(Math.round((q[3]*180/PI)*1000))/1000);
            
            
       
           
          float theta = 0;  
//float theta = acos((float)((qSave[5][1]*qSave[4][1]+qSave[5][2]*qSave[4][2]+qSave[5][3]*qSave[4][3])/(Math.sqrt(qSave[5][1]*qSave[5][1]+qSave[5][2]*qSave[5][2]+qSave[5][3]*qSave[5][3])*Math.sqrt(qSave[4][1]*qSave[4][1]+qSave[4][2]*qSave[4][2]+qSave[4][3]*qSave[4][3]))));
    // theta = acos((float)(   (quat[4].dot(quat[5])-qSave[4][0]*qSave[5][0])  /  (Math.sqrt(quat[4].dot(quat[4]))-qSave[4][0]*qSave[4][0])*(Math.sqrt(quat[5].dot(quat[5]))-qSave[5][0]*qSave[5][0])   ) );        
          //countMPU[teapotPacket[12]] ++;
          int currentMPU = int(teapotPacket[12]);
          for (int n = 0; n<4; n++) {
            qSave[currentMPU][n] = q[n];
          }
          for (int n = 0; n<3; n++ ){
            eulerSave[currentMPU][n] = ypr[n];
          }
          //Quaternion quat1 = new Quaternion(1, 0, 0, 0);
          quat[currentMPU].set(q[0], q[1], q[2], q[3]);
          
          //theta = ((float)(   (quat[4].dot(quat[5])-qSave[4][0]*qSave[5][0])  / (    Math.sqrt(quat[4].magnitude()*quat[4].magnitude()-qSave[4][0]*qSave[4][0])  *    Math.sqrt((quat[5].magnitude()*quat[5].magnitude()-qSave[5][0]*qSave[5][0]))   ) ));        
          
          
          println("\tMPU-" + currentMPU 
            + "\tnum" + num + "\tmillis: "+ millis() +"\tMPU0: " +countMPU[0]+"\tMPU1: " +countMPU[1]
            //+"\tMPU2: " +countMPU[2]+"\tMPU3: " +countMPU[3]+"\tMPU4: " +countMPU[4]+"\tMPU5: " +countMPU[5]
            +"\t0xFF-"+ byte(teapotPacket[11]) +"\tTheta-");

          
//
          if (currentMPU != MPUNumber - 1) {
            filled++;
          } else if(currentMPU == MPUNumber - 1) {
           
            if (filled != MPUNumber - 1) {
              //println("Not"+filled);
            } else if (filled == MPUNumber - 1) {
              
              if (Data2CsvFile.ENABLE)
              {
                newRow = table.addRow();
                //println(table.getRowCount());
                //newRow.setString("name", "Lion");
                //newRow.setString("type", "Mammal");
                for (int n = 0; n<MPUNumber; n++) {
                  for (int m = 0; m<4; m++) {
                    newRow.setString(title[n][m], String.valueOf(qSave[n][m]));
                  }
                }
              }
              
              if (Angle2CsvFile.ENABLE)
              {
                newRow = angleTable.addRow();
                //println(table.getRowCount());
                //newRow.setString("name", "Lion");
                //newRow.setString("type", "Mammal");
                for (int n = 0; n<MPUNumber; n++) {
                  for (int m = 0; m<3; m++) {
                    newRow.setString(angleTitle[n][m], String.valueOf(eulerSave[n][m]));
                  }
                }
              }
            }
            filled = 0;
          }

          }
        //}
      }
    }
  }
}

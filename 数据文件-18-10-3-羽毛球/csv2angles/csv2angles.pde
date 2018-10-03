//import processing.serial.*;
//import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;

Table quatTable;
Table axisTable;
Table angleOut;
Table caliTable;

String fileTime = "3-19-24-23";
String caliFileTime = "3-19-17-8";
int caliDis = 78; //从0到100；

int MPUNumber = 12;

Quaternion[] caliQuat = new Quaternion[MPUNumber];
Quaternion[] quat = new Quaternion[MPUNumber];
String[][] title = new String[MPUNumber][4];
Vec3D [] angleVec = new Vec3D[MPUNumber];
String[] angleOutTitle = new String[MPUNumber];
//float[] q = {1,0,0,0};
String axisFile = "../三轴角度/"+fileTime+".csv";
String quatFile = "../四元数/"+fileTime+".csv";
String caliFile = "../四元数/"+caliFileTime+".csv";
String outFile = "../关节角度解算输出/"+fileTime+".csv";

int minDiff = 1;


void getCali(TableRow caliRow){
  for (int n = 0; n<MPUNumber; n++) {
    caliQuat[n] = new Quaternion(1, 0, 0, 0);
    float[] q = new float[4];
    for (int m = 0; m<4; m++) {
      StringBuffer sbu = new StringBuffer();
      sbu.append((char)(65+n));
      String titleString = sbu.toString()+String.valueOf(m);
      print(titleString+" ");
      title[n][m]=titleString;
      q[m]= caliRow.getFloat(titleString);
      print(q[m]);
    }
    //for (int m = 0; m<4; m++) {
    //  print(q[m]);
    //}
    //Quaternion quatZ = new Quaternion(0, 0, 0, 1);
    //float w = q[0], x = q[1], y = q[2], z = q[3];
    //print("wxyz = "+w+x+y+z);
    //quat[n].set(w, x, y, z);
    caliQuat[n].set(q[0], q[1], q[2], q[3]);
  }
  println();
}

Quaternion reverseRotateWith(Quaternion Dir, Quaternion Rot){
  return ((Rot.getConjugate()).multiply(Dir)).multiply(Rot);
}

Quaternion rotateWith(Quaternion Dir, Quaternion Rot){
  return (Rot.multiply(Dir)).multiply(Rot.getConjugate());
}

Vec3D quat2Vec(Quaternion org, Quaternion cali, int dir){
  // dir = 1, 2, 3. 对应 x, y, z 方向。
  Quaternion quatDir;
  if (dir == 1) {
    quatDir = new Quaternion(0, 1, 0, 0);
  } else if (dir == 2) {
    quatDir = new Quaternion(0, 0, 1, 0);
  } else {
    quatDir = new Quaternion(0, 0, 0, 1);
  }
  Quaternion cali_reverse = new Quaternion(-cali.w, -cali.x, -cali.y, -cali.z);
  Quaternion quatA = rotateWith(quatDir, org);
  Quaternion quatB = reverseRotateWith(quatA,cali_reverse);
  Vec3D vec = new Vec3D(quatB.x, quatB.y, quatB.z);
  println("vec = "+vec.toString());
  return vec;
}

void saveCalcRow(TableRow angleOutRow,TableRow quatRow, int millisT){
  angleOutRow.setInt("millis", millisT);
  //得到此时原始四元数，赋值到
  for (int n = 0; n<MPUNumber; n++) {
    quat[n] = new Quaternion(1, 0, 0, 0);
    float[] q = new float[4];
    for (int m = 0; m<4; m++) {
      StringBuffer sbu = new StringBuffer();
      sbu.append((char)(65+n));
      String titleString = sbu.toString()+String.valueOf(m);
      //print(titleString+" ");
      //title[n][m]=titleString;
      q[m]= quatRow.getFloat(titleString);
      //print(q[m]);
    }
    quat[n].set(q[0], q[1], q[2], q[3]);
    angleVec[n] = quat2Vec(quat[n], caliQuat[n], 2);
  }
  angleVec[11] = quat2Vec(quat[11], caliQuat[11], 3); // 11号是唯一取Z方向的，方便计算角度。
//--------------------以下开始分类计算--------------------
  for (int i = 0;i< 5 ;i++ ) {
    float angle1 = angleVec[2*i].angleBetween(angleVec[2*i+1]);//计算五根手指指尖弯曲角度。
    angleOutRow.setFloat(angleOutTitle[i], angle1*180/PI);
  }
  for (int i = 0;i< 5 ;i++ ) {
    float angle1 = angleVec[10].angleBetween(angleVec[2*i+1]);//计算五根手指与手背夹角。
    angleOutRow.setFloat(angleOutTitle[i+5], angle1*180/PI);
  }
  float angle1 = angleVec[10].angleBetween(angleVec[11]);//计算手腕角度。
  angleOutRow.setFloat(angleOutTitle[10], angle1*180/PI);

}

void angleOutTableSetup(){
  angleOut = new Table();
  angleOut.addColumn("millis");
  for (int i = 0;i < MPUNumber ;i++ ) {
    StringBuffer sbu = new StringBuffer();
    sbu.append((char)(65+i));
    String titleString = sbu.toString();
    angleOutTitle[i] = titleString;
    angleOut.addColumn(titleString);
  }
}

void setup() {
  axisTable = loadTable(axisFile, "header");
  quatTable = loadTable(quatFile, "header");
  caliTable = loadTable(caliFile, "header");
  int axisRowCount = axisTable.getRowCount();
  int quatRowCount = quatTable.getRowCount();
  int caliRowCount = caliTable.getRowCount();

  int caliAt = caliRowCount*caliDis/100;
  int outRow = min(axisRowCount, quatRowCount);

  println("三轴角度\t总共 "+axisRowCount+" 行");
  println("四元数\t总共 "+quatRowCount+" 行");
  if(abs(axisRowCount - quatRowCount)>minDiff){
    println("数据量相差超过 "+minDiff);
    exit();
  }

  TableRow caliRow = caliTable.getRow(caliAt);

  getCali(caliRow);

  angleOutTableSetup();

  for (int i = 0;i < outRow ; i++) {
  	TableRow axisRow = axisTable.getRow(i);
  	int millisT = axisRow.getInt("millis");
    TableRow quatRow = quatTable.getRow(i);
  	//println(millisT);
    TableRow angleOutRow = angleOut.addRow();
    saveCalcRow(angleOutRow, quatRow, millisT);
  }
  saveTable(angleOut, outFile);
  exit();
}

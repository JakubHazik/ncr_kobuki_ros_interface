
//#include "stdafx.h"
#include <ncr_kobuki_ros_interface/file_lidar_sim.h>

#include "errno.h"
#include "string.h"

//--pripoji sa na subor s ulozenym meranim...
int rplidar::connectToFile(char *filename)
{

    fp=fopen(filename,"r+");
	return 0;
}
//ziska meranie z filu
LaserMeasurement rplidar::getMeasurementFromFile()
{
    LaserMeasurement tempL;
    if(fp==NULL)
    {
        tempL.numberOfScans=-2;
        return tempL;
    }
    char text[5000];
    char *tt=fgets(text,5000,fp);

    if(tt==NULL)
    {
        tempL.numberOfScans=-1;
        return tempL;
    }
    char *txt=(char*)calloc(strlen(text)+1,sizeof(char));
    strcpy(txt,text);
    txt=strtok(txt,"\t ");
    tempL.timestamp=atoi(txt);
    txt=strtok(NULL,"\t ");
    tempL.numberOfScans=atoi(txt);
    tt=fgets(text,5000,fp);
    txt=(char*)calloc(strlen(text)+1,sizeof(char));
    strcpy(txt,text);
    txt=strtok(txt,"\t ");
    for(int i=0;i<tempL.numberOfScans;i++)
    {
        tempL.Data[i].scanDistance=atof(txt);
        txt=strtok(NULL,"\t ");
    }
    tt=fgets(text,5000,fp);
    txt=(char*)calloc(strlen(text)+1,sizeof(char));
        strcpy(txt,text);
    txt=strtok(txt,"\t ");
    for(int i=0;i<tempL.numberOfScans;i++)
    {
        tempL.Data[i].scanAngle=atof(txt);
        txt=strtok(NULL,"\t ");
    }
//    for(int i=0;i<tempL.numberOfScans;i++)
//    {
//        printf("uhol %f vzdialenost %f\n",tempL.Data[i].scanAngle,tempL.Data[i].scanDistance);
//    }

    return tempL;


}



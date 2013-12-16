#include "fancyViewer.h"

void fancyViewer::draw()
{

    glPointSize(2.0f);
    glDisable(GL_CULL_FACE);
    sensor_msgs::PointCloud c1;
    sensor_msgs::PointCloud c2;
    mymap m;
    drawPointCloud(cloud1,0.0f,1.0f,0.0f);
    drawPointCloud(cloud2,1.0f,0.0f,0.0f);
    drawAssociations(cloud1, cloud2, mapPointer,1.0f,1.0f,0.0f);


}

void fancyViewer::drawPointCloud(sensor_msgs::PointCloud &c, float r, float g, float b)
{
    glBegin(GL_POINTS);
    geometry_msgs::Point32 p;
    glColor3f(r,g,b);

    for(unsigned int i=0;i<c.points.size();i++)
    {
        p=c.points.at(i);
        glVertex3f(p.x,p.y,p.z);
    }
    glColor3f(1,1,1);
    glEnd();
}

void fancyViewer::drawAssociations(sensor_msgs::PointCloud &sourceCloud, sensor_msgs::PointCloud &destinationCloud, mymap &fancyMap,float r,float g,float b)
{
    glBegin(GL_LINES);
    geometry_msgs::Point32 vertex;

    mymap::iterator i = fancyMap.begin();
    while (i!=fancyMap.end()){
        glColor3f(r,g,b);
        std::pair<int,mapData> pair=(*i);
        mapData k=pair.second;
        if((unsigned int)pair.first<(unsigned int)sourceCloud.points.size() && (unsigned int)k.first<(unsigned int)destinationCloud.points.size()){ //debug
        vertex=sourceCloud.points.at(pair.first);
        glVertex3f(vertex.x,vertex.y,vertex.z);
        vertex=destinationCloud.points.at(k.first);
        glVertex3f(vertex.x,vertex.y,vertex.z);
        }
        i++;
        glColor3f(1,1,1);
    }
    glEnd();

}

void fancyViewer::init()
{
    setGridIsDrawn();
    startAnimation();
    setSceneRadius(10);
    camera()->showEntireScene();
    //this->p=0;
}

fancyViewer::fancyViewer(QWidget *parent) : QGLViewer(parent)
{
    printf("-> Viewer initialized\n");
}

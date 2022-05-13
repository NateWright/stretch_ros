#include "mapsubscriber.hpp"

mapsubscriber::mapsubscriber(ros::NodeHandle* nodeHandle) : nh_(nodeHandle) {
    mapSub_ = nh_->subscribe("/map", 30, &mapsubscriber::callback, this);
}

void mapsubscriber::run() {
    ros::Rate loop_rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        emit mapUpdate(img_);

        loop_rate.sleep();
    }
}

void mapsubscriber::callback(const nav_msgs::OccupancyGrid msg) {
    //  img_ = QPixmap(msg.info.width, msg.info.height);
    //  qb = QByteArray();
    //  for(int i : msg.data){
    //   qb.push_back(i);
    //  }
    //  ROS_INFO_STREAM(qb.size());
    //  img_ = QPixmap::fromImage(QImage((uchar*)qb.data(), msg.info.width, msg.info.height, QImage::Format_RGB888));

    QImage q(msg.info.width, msg.info.height, QImage::Format_RGB888);

    int val = 0;
    int pos = 0;
    for (int y = 0; y < msg.info.height; y++) {
        for (int x = 0; x < msg.info.width; x++) {
            pos = x + msg.info.width * y;
            if (msg.data[pos] == -1) {
                val = 100;
            } else if (msg.data[pos] == 100) {
                val = 0;
            } else {
                val = 255;
            }
            q.setPixel(x, y, QColor(val, val, val).rgb());
        }
    }

    img_ = QPixmap::fromImage(q);
}

mapsubscriber::~mapsubscriber() {
    delete data;
}

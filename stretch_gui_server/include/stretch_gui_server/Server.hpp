#ifndef SERVER_HPP
#define SERVER_HPP

#include <QBuffer>
#include <QByteArray>
#include <QObject>
#include <QPixmap>
#include <QPoint>
#include <QSize>
#include <QTimer>
#include <stretch_gui_library/StretchMoveItInterface.hpp>
#include <stretch_gui_library/graspnode.hpp>
#include <stretch_gui_library/mapsubscriber.hpp>
#include <stretch_gui_library/movebasestatus.hpp>
#include <stretch_gui_library/roscamera.hpp>
#include <stretch_gui_library/sceneviewer.hpp>

#include "rep_Server_source.h"

class Server : public ServerSimpleSource {
    Q_OBJECT
   public:
    explicit Server(QObject *parent = nullptr);
    ~Server();

    void uiButtonGraspClicked();
    void uiButtonStopClicked();
    void uiButtonSetHomeClicked();
    void uiButtonNavigateHomeClicked();
    void uiButtonToggleNavTypeClicked();
    void uiDisplayMapMouseClick(QPoint press, QPoint release, QSize screen);
    void uiDisplayMapMousePressInitiated(QPoint press, QSize screen);
    void uiDisplayMapMousePressCurrentLocation(QPoint loc, QSize screen);
    void uiButtonBackClicked();
    void uiCameraMoveButtonUpClicked();
    void uiCameraMoveButtonDownClicked();
    void uiCameraMoveButtonLeftClicked();
    void uiCameraMoveButtonRightClicked();
    void uiCameraMoveButtonHomeClicked();
    void uiDisplayCameraMouseClicked(QPoint press, QPoint release, QSize screen);

    void uiConfirmButtonNoClicked();
    void uiConfirmButtonYesClicked();

    void uiButtonBack_2Clicked();
    void uiButtonReturnObjectClicked();
    void uiButtonReleaseClicked();
    void uiButtonReplaceObjectClicked();
    void uiButtonNavigateClicked();

    void uiButtonBackToGraspClicked();

   private:
    ros::NodeHandlePtr nh_;
    MapSubscriber *mapNode_;
    MoveBaseStatus *moveBaseStatusNode_;
    RosCamera *cameraNode_;
    GraspNode *graspNode_;
    StretchMoveItInterface *moveItNode_;

    void initConnections();

    QMetaObject::Connection cameraNodeImgUpdate_;
    QMetaObject::Connection cameraNodeImgUpdateWP_;

   signals:
    void enableMapping();
    void disableMapping();
    void homeRobot();
    void cameraSetTilt(int);
    void ButtonGraspClicked();
    void ButtonStopClicked();
    void ButtonSetHomeClicked();
    void ButtonNavigateHomeClicked();
    void ButtonToggleNavTypeClicked();
    void DisplayMapMouseClick(QPoint press, QPoint release, QSize screen);
    void DisplayMapMousePressInitiated(QPoint press, QSize screen);
    void DisplayMapMousePressCurrentLocation(QPoint loc, QSize screen);

    void ButtonBackClicked();
    void CameraMoveButtonUpClicked();
    void CameraMoveButtonDownClicked();
    void CameraMoveButtonLeftClicked();
    void CameraMoveButtonRightClicked();
    void CameraMoveButtonHomeClicked();
    void DisplayCameraMouseClicked(QPoint press, QPoint release, QSize screen);

    void ConfirmButtonNoClicked();
    void ConfirmButtonYesClicked();

    void ButtonBack_2Clicked();
    void ButtonReturnObjectClicked();
    void ButtonReleaseClicked();
    void ButtonReplaceObjectClicked();
    void ButtonNavigateClicked();

    void ButtonBackToGraspClicked();


   private slots:
    void changeToPage1();
    void changeToPage2();
    void changeToPage3();
    void changeToPage4();
    void changeToPage5();
    void changeToPage6();
};

#endif  // SERVER_HPP

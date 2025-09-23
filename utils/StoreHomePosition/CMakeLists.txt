add_application(
    NAME StoreHomePosition
    SOURCES src/main.cpp src/StoreHomePosition.cpp
    HEADERS include/StoreHomePosition/StoreHomePosition.h
    LINK_LIBRARIES CiA402::CiA402 YARP::YARP_os YARP::YARP_dev)
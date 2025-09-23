add_application(
    NAME yarp-CiA402-store-home-position
    SOURCES src/main.cpp src/StoreHomePosition.cpp
    HEADERS include/StoreHomePosition/StoreHomePosition.h
    LINK_LIBRARIES CiA402::EthercatManager YARP::YARP_os YARP::YARP_dev)

add_application(
    NAME store-home-position
    SOURCES src/main.cpp src/StoreHomePosition.cpp
    HEADERS include/StoreHomePosition/StoreHomePosition.h
    LINK_LIBRARIES YarpCiA402::EthercatManager YARP::YARP_os YARP::YARP_dev)

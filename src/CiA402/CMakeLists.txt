add_cia402_library(
    NAME CiA402  
    SOURCES src/EthercatManager.cpp
    PUBLIC_HEADERS include/CiA402/EthercatManager.h
    PUBLIC_LINK_LIBRARIES soem
)
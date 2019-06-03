add_executable(network_ctl
network_ctl_linux.cpp
../spi_proto/src/spi_proto_master_datagram.cpp
../spi_proto/src/spi_proto.c
../spi_proto/src/crc16.c)
target_link_libraries(network_ctl pthread)
target_include_directories(network_ctl PUBLIC "../spi_proto/src/")

#include "data_handler.hpp"

void DataHandler::init()
{
	std::string ip_addr; 
    int numTry = 0;

    // Load Radar IP address
    this->declare_parameter<std::string>("ip", ip_addr);
    this->get_parameter("ip", ip_addr);
    RCLCPP_INFO(this->get_logger(), "IP Address : %s", ip_addr.c_str());

    // Set the height of radar
    this->declare_parameter<int>("z_offset", z_offset);
    this->get_parameter("z_offset", z_offset);
    RCLCPP_INFO(this->get_logger(), "Z Offset : %d", z_offset);

    while(!m_socket.connectSocket(ip_addr.c_str(), RADAR_DATA_RX_PORT)) {
        numTry++;
        if(numTry <= 20) { // 20 tries
            usleep(100000);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Tried 20 times, but couldn't connect. Bye !");
            return;
        }
    }

    // Set the basic properties of pointcloud 
    cloud2.header.frame_id = "retina_link";
    cloud2.height = 1;
    cloud2.fields.resize(5);
    // Convert x/y/z to fields
    cloud2.fields[0].name = "x"; 
    cloud2.fields[1].name = "y"; 
    cloud2.fields[2].name = "z";
    cloud2.fields[3].name = "power";
    cloud2.fields[4].name = "doppler";

    int offset = 0;
    // All offsets are *4, as all field data types are float32
    for (int d = 0; d < cloud2.fields.size(); ++d, offset += 4) {
        cloud2.fields[d].offset = offset;
        cloud2.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud2.fields[d].count = 1;
    }

    cloud2.point_step = offset;
    cloud2.data.reserve(MAX_NUM_POINTS_PER_FRAME * cloud2.point_step);
    cloud2.is_bigendian = false;  
    cloud2.is_dense = false;
}

int8_t DataHandler::receive()
{
    packet_buffer_t packetBuffer;
	int readBytes = 0;
    static int format_type = 0xFF;
	
    readBytes = m_socket.readData((uint8_t *)&packetBuffer.cmdHeader, RADAR_CMD_HEADER_LENGTH, true);
    if(readBytes == RADAR_CMD_HEADER_LENGTH) {
        if(memcmp(&packetBuffer.cmdHeader.header, NETWORK_TX_HEADER, NETWORK_TX_HEADER_LENGTH) != 0) {
           RCLCPP_INFO(this->get_logger(), "Not match with the TI header magic number !!!");
            return 0;
        }

        if(packetBuffer.cmdHeader.dataSize > MAX_BUF_SIZE) {
            RCLCPP_ERROR(this->get_logger(), "Greater than max buffer size !");
            return 0;
        }
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Read bytes(%d) is not matching the data size !!!", readBytes);
		return 0;
    }

    readBytes = m_socket.readData((uint8_t *)&packetBuffer.buf, FRAME_SIZE, true);
    if (readBytes == FRAME_SIZE)
    {
        if(memcmp(packetBuffer.pkHeader.magicWord, radarMagicWord, RADAR_OUTPUT_MAGIC_WORD_LENGTH) != 0)
        {
            RCLCPP_DEBUG(this->get_logger(), "Magic Word is not matched !!!");
            return 0;
        }

        // Frame Number
        frame_number = packetBuffer.pkHeader.frame_counter;
			
        // The number of points
        nPoints = packetBuffer.pkHeader.targetNumber;

        // Target Format Type -> 1 : Spherical, 2 : Cartesian
        if(format_type == 0xFF) {
            format_type = packetBuffer.pkHeader.target_info_format_type;
            RCLCPP_INFO(this->get_logger(), "Format type : %d", format_type);
        }

        // Just for your information
        RCLCPP_DEBUG(this->get_logger(), "Frame Counter : %d, Total number of points : %d", frame_number, nPoints);

        for (uint32_t i = 0; i < nPoints; i++)
        {
            point_data_t    pt;
            rcv_data_t      *point = (rcv_data_t *)(packetBuffer.data + (sizeof(rcv_data_t) * i));

            if(format_type == 1) {                  // Spherical Coordinate
                float range     = point->data[0];
                float azimuth   = point->data[1];
                float elevation = point->data[2];
                pt.x            = polar2x(range, azimuth, elevation);
                pt.y            = polar2y(range, azimuth, elevation);
                pt.z            = polar2z(range, azimuth, elevation);
                pt.doppler      = point->doppler;
                pt.power        = point->power;
            } else if(format_type == 2) {           // Cartesian Coordinate
                pt.x            = point->data[0];
                pt.y            = point->data[1];
                pt.z            = point->data[2];   
                pt.doppler      = point->doppler;
                pt.power        = point->power;                
            } else {
                RCLCPP_INFO(this->get_logger(), "Not defined format type !");
                return -1;
            }         

            r_data[i] = pt;
            r_data[i].z += (z_offset/100.f);
			
			// just for debugging
            RCLCPP_DEBUG(this->get_logger(), "[%d] (%.4f, %.4f, %.4f)", i, pt.x, pt.y, pt.z);
        }
	}

    return 1;
}

int8_t DataHandler::publish()
{
    cloud2.header.stamp = rclcpp::Clock().now();
    cloud2.width = nPoints;
    cloud2.row_step = cloud2.point_step * cloud2.width;
    cloud2.data.resize(nPoints * cloud2.point_step * cloud2.height);

    // Copy the data points
    for (int cp = 0; cp < nPoints; ++cp) {
        memcpy(&cloud2.data[cp * cloud2.point_step + cloud2.fields[0].offset], &r_data[cp].x,       sizeof(float));
        memcpy(&cloud2.data[cp * cloud2.point_step + cloud2.fields[1].offset], &r_data[cp].y,       sizeof(float));
        memcpy(&cloud2.data[cp * cloud2.point_step + cloud2.fields[2].offset], &r_data[cp].z,       sizeof(float));
        memcpy(&cloud2.data[cp * cloud2.point_step + cloud2.fields[3].offset], &r_data[cp].power,   sizeof(float));
        memcpy(&cloud2.data[cp * cloud2.point_step + cloud2.fields[4].offset], &r_data[cp].doppler, sizeof(float));
    }

    publisher_->publish(cloud2);

	return 1;
}

// Change Polar Coordinate to Cartesian Coordinate
float DataHandler::polar2x(float range, float azimuth, float elevation)
{
    float phi = azimuth * pi / 180.0f;          // Azimuth angle(radian)
    float theta = elevation * pi / 180.0f;      // Elevation angle(radian)

    return (range * cosf(theta) * sinf(phi));
}

float DataHandler::polar2y(float range, float azimuth, float elevation)
{
    float phi = azimuth * pi / 180.0f;          // Azimuth angle(radian)
    float theta = elevation * pi / 180.0f;      // Elevation angle(radian)

    return (range * cosf(theta) * cosf(phi));
}

float DataHandler::polar2z(float range, float azimuth, float elevation)
{
    float phi = azimuth * pi / 180.0f;          // Azimuth angle(radian)
    float theta = elevation * pi / 180.0f;      // Elevation angle(radian)

    return (range * sinf(theta));
}

namespace{
  /**
   * @brief for lidar .bin file read / 读取.bin文件的点云,
   * 
   * @tparam T typename
   * @param lidar_data_path 
   * @return std::vector<T>  x, y, z, intensity order to std::vector<T> / 点云按照 x, y, z, intensity 的顺序存进std::vector<T>
   */
  template <typename T>
  static std::vector<T> read_lidar_data(const std::string lidar_data_path)
  {
      std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
      lidar_data_file.seekg(0, std::ios::end);
      const size_t num_elements = lidar_data_file.tellg() / sizeof(T);
      lidar_data_file.seekg(0, std::ios::beg);

      std::vector<T> lidar_data_buffer(num_elements);
      lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(T));
      return lidar_data_buffer;
  }
}
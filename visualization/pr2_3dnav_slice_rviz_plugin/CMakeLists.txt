
# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES include/pr2_3dnav_slice_rviz_plugin/pr2_3dnav_slice_display.h)

set(PR2_3DNAV_LIB_NAME pr2_3dnav_slice_rviz_plugin)
add_library(${PR2_3DNAV_LIB_NAME}_core src/pr2_3dnav_slice_display.cpp ${MOC_SOURCES})
target_link_libraries(${PR2_3DNAV_LIB_NAME}_core ${catkin_LIBRARIES} ${OGRE_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES})

add_library(${PR2_3DNAV_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${PR2_3DNAV_LIB_NAME} ${PR2_3DNAV_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PR2_3DNAV_LIB_NAME}_core ${PR2_3DNAV_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


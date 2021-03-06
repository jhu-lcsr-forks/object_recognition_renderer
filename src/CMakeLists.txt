FIND_PACKAGE(OpenGL)

find_package(PkgConfig REQUIRED)
find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
  pkg_check_modules(ASSIMP assimp)
endif()
if (NOT ASSIMP_FOUND)
  message(WARNING "ASsimp not found, not building synthetic views")
endif()
if( ${ASSIMP_VERSION} STRGREATER "2.0.0" )
  message(STATUS "Found assimp v3")
  set(EXTRA_SOURCES)
  set(EXTRA_LIBRARIES assimp)
else()
  message(STATUS "Building assimp v3")
  set(ASSIMP_INCLUDE_DIRS ./assimp/include)
  aux_source_directory(./assimp/contrib/clipper EXTRA_SOURCES_clipper)
  aux_source_directory(./assimp/contrib/ConvertUTF EXTRA_SOURCES_ConvertUTF)
  aux_source_directory(./assimp/contrib/irrXML EXTRA_SOURCES_irrXML)
  aux_source_directory(./assimp/contrib/poly2tri/poly2tri/common EXTRA_SOURCES_poly2tri)
  aux_source_directory(./assimp/contrib/poly2tri/poly2tri/sweep EXTRA_SOURCES_poly2tri_sweep)
  aux_source_directory(./assimp/contrib/unzip EXTRA_SOURCES_unzip)
  aux_source_directory(./assimp/contrib/zlib EXTRA_SOURCES_zlib)
  aux_source_directory(./assimp/code EXTRA_SOURCES)
  set(EXTRA_SOURCES ${EXTRA_SOURCES} ${EXTRA_SOURCES_clipper} ${EXTRA_SOURCES_ConvertUTF} ${EXTRA_SOURCES_irrXML} ${EXTRA_SOURCES_poly2tri} ${EXTRA_SOURCES_poly2tri_sweep} ${EXTRA_SOURCES_unzip} ${EXTRA_SOURCES_zlib})
  set(EXTRA_LIBRARIES)
endif()


include_directories(BEFORE ${ASSIMP_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(SYSTEM ${OpenCV_INCLUDE_DIR}
                           ${OPENGL_INCLUDE_DIR}
)

if (${USE_GLUT})
  find_package(GLUT)
  include_directories(SYSTEM ${GLUT_INCLUDE_DIR})

  # add a glut version of the renderer
  add_library(${PROJECT_NAME}_glut model.cpp
                                   renderer.cpp
                                   renderer_glut.cpp
                                   ${EXTRA_SOURCES}
  )
  set_target_properties(${PROJECT_NAME}_glut PROPERTIES COMPILE_FLAGS "-DUSE_RENDERER_GLUT=1")
  target_link_libraries(${PROJECT_NAME}_glut ${EXTRA_LIBRARIES}
                                             ${OpenCV_LIBRARIES}
                                             ${OPENGL_LIBRARIES}
                                             ${GLUT_LIBRARIES}
                                             freeimage
                                             boost_program_options
                                             boost_system
                                             boost_filesystem
  )
  if (ROS_GROOVY_FOUND)
  install(TARGETS ${PROJECT_NAME}_glut
          DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
  else()
  install(TARGETS ${PROJECT_NAME}_glut
          DESTINATION lib
  )
  endif()
endif()

if(${USE_OSMESA})
  # add an OSMesa version of the renderer
  add_library(${PROJECT_NAME}_osmesa model.cpp
                                     renderer.cpp
                                     renderer_osmesa.cpp
                                     ${EXTRA_SOURCES}
  )
  set_target_properties(${PROJECT_NAME}_osmesa PROPERTIES COMPILE_FLAGS "-DUSE_RENDERER_OSMESA=1")
  target_link_libraries(${PROJECT_NAME}_osmesa ${EXTRA_LIBRARIES}
                                               ${OpenCV_LIBRARIES}
                                               OSMesa GLU
                                               freeimage
                                               boost_program_options
                                               boost_system
                                               boost_filesystem
  )
  if (ROS_GROOVY_FOUND)
  install(TARGETS ${PROJECT_NAME}_osmesa
          DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
  else()
  install(TARGETS ${PROJECT_NAME}_osmesa
          DESTINATION lib
  )
  endif()
endif()

ADD_EXECUTABLE(view_generator main.cpp)

SET_PROPERTY(TARGET view_generator PROPERTY DEBUG_POSTFIX ${ASSIMP_DEBUG_POSTFIX})

TARGET_LINK_LIBRARIES(view_generator ${EXTRA_LIBRARIES})

if (${USE_GLUT})
  set_target_properties(view_generator PROPERTIES COMPILE_FLAGS "-DUSE_RENDERER_GLUT=1")
  target_link_libraries(view_generator ${PROJECT_NAME}_glut)
else()
  target_link_libraries(view_generator ${PROJECT_NAME}_osmesa)
endif()

SET_TARGET_PROPERTIES(view_generator PROPERTIES
                                     OUTPUT_NAME view_generator
)

if (ROS_GROOVY_FOUND)
install(TARGETS view_generator
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
else()
install(TARGETS view_generator
        DESTINATION bin
)
endif()

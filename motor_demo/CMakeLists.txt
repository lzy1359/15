cmake_minimum_required(VERSION 3.8)

project(motor_demo)

if (LVGL_V9)
	MESSAGE(FATAL_ERROR "This app no support LVGL9, please switch to LVGL8")
endif()

include_directories(${PROJECT_SOURCE_DIR}/)

link_directories(./)

set(MOTOR_DEMO_SRCS
	${SRCS}
	main.c
	ml_label.c
	motor_control_client.c
)

if(ROCKCHIP_IGH_ETHERCAT_DIR)
	include_directories(
		${ROCKCHIP_IGH_ETHERCAT_DIR}/demo
		${CMAKE_SYSROOT}/usr/include
	)
endif()

if(ROCKCHIP_LINUX_RGA_DIR)
	include_directories(
		${ROCKCHIP_LINUX_RGA_DIR}/im2d_api
		${ROCKCHIP_LINUX_RGA_DIR}/include
	)
endif()

add_executable(${PROJECT_NAME} ${MOTOR_DEMO_SRCS})

target_link_libraries(${PROJECT_NAME}
	rga
)

if(ROCKCHIP_IGH_ETHERCAT_DIR)
	add_executable(motor_control_daemon
		motor_control_daemon.c
		${ROCKCHIP_IGH_ETHERCAT_DIR}/demo/Rockchip_MADHT1505BA1.c
	)

	target_link_libraries(${PROJECT_NAME}
		rt
	)

	target_link_libraries(motor_control_daemon
		ethercat
		rt
	)
else()
	target_link_libraries(${PROJECT_NAME}
		rt
	)
endif()

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
if(TARGET motor_control_daemon)
	install(TARGETS motor_control_daemon RUNTIME DESTINATION bin)
endif()
install(FILES
	resource/SmileySans-Oblique.ttf
	resource/needle.png
	DESTINATION share/resource
)
install(PROGRAMS S10motor_demo DESTINATION /etc/init.d/pre_init)
install(PROGRAMS S60motor_demo DESTINATION /etc/init.d)
install(PROGRAMS motor_stack_ctl DESTINATION /usr/bin)

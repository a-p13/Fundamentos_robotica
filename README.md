Los proyectos fueron realizados en wsl (porque se uso ros2), pero pueden ser corridos en linux. Los archivos que se agregan aqui, solo son los que estan en la carpeta de src del proyecto. Para crear la carpeta completa de cada challenge, copia y pega estos comandos en tu terminal:
mkdir <nombreCarpeta>
cd <nombreCarpeta>
mkdir src
cd src/
ros2 pkg create  --build-type ament_python  --node-name talker  basic_comms --dependencies rclpy std_msgs

Para correr el archivo, poner en la terminal:
- Colcon build
- source install/setup.bash
- ros2 run <carpeta> <archivo>

Como este proyecto lo realizamos para la esp32, tuvimos que agregarle unas cosas a la IDE para que reconociera la tarjeta. 
- File > Preferences > Additional boards manager URLs: http://arduino.esp8266.com/stable/package_esp8266com_index.json
- Despues, buscar la libreria de esp32

Para instalar Mico-ros, siga estas instrucciones: 
![image](https://github.com/a-p13/Fundamentos_robotica/assets/88742201/cfaaa009-c231-4ad9-9d5b-69c4619e4925)
![image](https://github.com/a-p13/Fundamentos_robotica/assets/88742201/8a46f3c0-9d1f-463e-94cf-179c4a0c8c7a)


Link para poder darle control de tu puerto usb a wsl y poder linkear micro ros y ros2: https://learn.microsoft.com/es-es/windows/wsl/connect-usb#attach-a-usb-device.

Comando para correr en terminal en caso de querer probar el challenge 3:
![image](https://github.com/a-p13/Fundamentos_robotica/assets/88742201/807faf08-e9ed-4e09-81b1-f14b5e9c10bd)}


Para mas informacion sobre los retos: https://github.com/ManchesterRoboticsLtd/TE3001B_Robotics_Foundation_2024.git


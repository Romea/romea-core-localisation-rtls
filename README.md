## Romea Core Localisation RTLS Library

This library provides a plugin that transforms data from an RTLS transceiver into Gaussian observations (position, pose, range), enabling seamless integration with the localization algorithms in the **romea_core_localisation** library. Additionally, it includes monitoring tools to track RTLS data and generate diagnostic reports, helping to detect and troubleshoot potential issues in real-time.

## **Usage**

1. create a ROS workspace
2. cd worskpace
3. mkdir src
4. wget https://raw.githubusercontent.com/Romea/romea-core-localisation-rtls/refs/heads/main/romea_localisation_rtls_public.repos
5. vcs import src < romea_localisation_rtls_public.repos
6. build packages
   - catkin build for ROS1
   - colcon build for ROS2
7. create your application using this library

## **Contributing**

If you'd like to contribute to this library, here are some guidelines:

1. Fork the repository.
2. Create a new branch for your changes.
3. Make your changes.
4. Write tests to cover your changes.
5. Run the tests to ensure they pass.
6. Commit your changes.
7. Push your changes to your forked repository.
8. Submit a pull request.

## **License**

This project is released under the Apache License 2.0. See the LICENSE file for details.

## **Authors**

The Romea Core Localization RTLS library, written by **Jean Laneurit**, was developed during ANR Baudet Rob 2 and ANR ADAP2E projects. Several individuals contributed scientifically to the development of this library:

**Jean Laneurit**  
**Christophe Debain**  
**Roland Chapuis**  
**Romuald Aufrere**  

## **Contact**

If you have any questions or comments about Romea Core Localisation RTLS library, please contact **[Jean Laneurit](mailto:jean.laneurit@inrae.fr)**.
# System Functional Test

La idea es mantener dentro de este paquete de python distintos test funcionales
para distintas acciones en el **sistema de navegación** de los ROV. 
El sistema está compuesto por una capa de comunicación, capa de firmware y capa 
de hardware (mavlink, ardusub y pixhawk respectivamente).

Algunos lineamientos a seguir:

* La idea es que cada script sea una guía para exportar el código a producción
* Mantener una funcionalidad mínima y responsabilidad clara
* No es necesario que el código mantenga criterios de modularidad o escalabilidad


*New Branch to port to LibGDX*

To incorporate in an Android Studio/IntelliJ libgdx project create a libs directory under core, clone this project under libs,
and then add the necessary src dirs to your core gradle build file . e.g

sourceSets.main.java.srcDirs = [ "src/","libs/RubeLoader/RubeLoader/src","libs/b2dJson/java/src/main/java" ]

You might like to mark the other directories as "excluded" in the intelliJ IDE - I have zero idea about the effects of that
when using a gradle build.

b2dJson
=======

Utilities to load scenes created by the R.U.B.E Box2D editor.

Main info page is at: http://www.iforce2d.net/b2djson

Please also see the usage examples here: http://www.iforce2d.net/rube/?panel=loaders

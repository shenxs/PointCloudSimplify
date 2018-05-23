#version 330 core

in vec3 ourColor;

uniform vec4 color;
uniform sampler2D qt_Texture0;

out vec4 FragColor;

void main(void)
{
//    gl_FragColor = texture2D(qt_Texture0, qt_TexCoord0.st);
    FragColor=vec4(0.0,0.0,0.0,1.0);
}

#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 color;
out vec3 ourColor;

uniform mat4 projMatrix;
uniform mat4 viewMatrix;
uniform mat4 modelMatrix;

void main(void)
{
    gl_Position = projMatrix* viewMatrix* modelMatrix*vec4(aPos,1.0);
    ourColor=color;
}

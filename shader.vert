#version 330 core

uniform mat4 projMatrix;

uniform mat4 viewMatrix;

uniform mat4 modelMatrix;

layout (location = 0) in vec3 aPos;


void main(void)
{
    gl_Position = projMatrix* viewMatrix* modelMatrix*vec4(aPos,1.0);
}

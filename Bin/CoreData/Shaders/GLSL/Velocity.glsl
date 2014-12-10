#include "Uniforms.glsl"
#include "Samplers.glsl"
#include "Transform.glsl"

varying vec3 vTexCoord;
varying vec4 vPosition;
varying vec4 vPrevPosition;
varying vec4 vPrevScreenPos;

void VS()
{
    mat4 modelMatrix = iModelMatrix;
    vec3 worldPos = GetWorldPos(modelMatrix);
    vec3 prevWorldPos = GetPrevWorldPos();
    gl_Position = GetClipPos(worldPos);
    vPosition = gl_Position;
    vPrevPosition = GetPrevClipPos(prevWorldPos);
    vPrevScreenPos = GetPrevClipPos(worldPos);
    vTexCoord = vec3(GetTexCoord(iTexCoord), GetDepth(gl_Position));
}

void PS()
{
    #ifdef ALPHAMASK
        float alpha = texture2D(sDiffMap, vTexCoord.xy).a;
        if (alpha < 0.5)
            discard;
    #endif

    vec2 curr = (vPosition.xy / vPosition.w) * 0.5 + 0.5;
    vec2 prev = (vPrevPosition.xy / vPrevPosition.w) * 0.5 + 0.5;
    vec2 velocity = (curr - prev);

    gl_FragColor = vec4(velocity * 0.5 + 0.5, 0.0, 1.0);
}

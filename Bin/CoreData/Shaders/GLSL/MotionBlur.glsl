#include "Uniforms.glsl"
#include "Samplers.glsl"
#include "Transform.glsl"
#include "ScreenPos.glsl"

uniform float cSamples;

varying vec2 vScreenPos;

void VS()
{
    mat4 modelMatrix = iModelMatrix;
    vec3 worldPos = GetWorldPos(modelMatrix);
    gl_Position = GetClipPos(worldPos);
    vScreenPos = GetScreenPosPreDiv(gl_Position);
}

void PS()
{
    vec2 blurVec = texture2D(sEnvMap, vScreenPos).rg;
    blurVec = blurVec * 2.0 - 1.0;

    // Viewport texture
    vec4 result = texture2D(sDiffMap, vScreenPos);

    for (int i = 1; i < cSamples; ++i) 
    {
        // Get offset in range [-0.5, 0.5]:
        vec2 offset = blurVec * (float(i) / float(cSamples - 1) - 0.5);
    
        // Sample & add to result:
        result += texture2D(sDiffMap, vScreenPos + offset);
    }
    result /= float(cSamples);

    gl_FragColor = result;
}

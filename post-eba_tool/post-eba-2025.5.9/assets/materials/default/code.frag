const vec2 texCoord = getTexCoord0();

material.baseColor.rgb = vec3(1, 0.9, 0.9);

//const vec3 N = sampleNormalMap(t_Normal, texCoord);
//material.normal = tangentToWorld(N, texCoord);

material.metallic = 0.7;
material.roughness = 0.5;

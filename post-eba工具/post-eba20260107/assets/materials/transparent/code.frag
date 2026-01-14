const vec2 texCoord = getTexCoord0();

material.baseColor.rgb = sRGBToLinear(texture(t_Diffuse, texCoord).rgb);

material.metallic = 0.9;
material.roughness = 0.2;

material.opacity = 0.35;

#version 330 core
in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoords;

out vec4 FragColor;

// uniform vec4 diffusecol;  // Diffuse color
// uniform vec4 specularcol; // Specular color
// uniform vec3 lightDirection; // Direction of the light source
// uniform vec3 viewDirection;  // Direction of the camera/viewer


uniform vec4 inputColor; // Input color for the fragment

void main() {
    // vec3 normal = normalize(Normal);
    // vec3 lightDir = normalize(lightDirection); // Use the uniform light direction
    // float diff = max(dot(normal, lightDir), 0.0);
    // vec4 diffuse = diff * diffusecol;
    // vec4 specular = vec4(0.0);

    // // Calculate specular component using the view direction
    // if (diff > 0.0) {
    //     vec3 viewDir = normalize(viewDirection); // Use the uniform view direction
    //     vec3 reflectDir = reflect(-lightDir, normal);
    //     float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0); // Shininess factor (adjust as needed)
    //     specular = spec * specularcol;
    // }

    // FragColor = inputColor * (diffuse + specular); // Final pixel color with inputColor, diffuse, and specular lighting
    FragColor = inputColor;
}

#pragma once

#include <glad/glad.h>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <iostream>
#include <string>

struct TextureParams
{
    GLuint wrapS = GL_CLAMP_TO_EDGE;
    GLuint wrapT = GL_CLAMP_TO_EDGE;
    GLuint filterMin = GL_NEAREST_MIPMAP_LINEAR;
    GLuint filterMax = GL_NEAREST;
};

class Texture2D
{
public:
    GLuint ID;
    GLuint Width, Height;
    GLuint InternalFormat, ImageFormat;
    GLuint WrapS, WrapT;
    GLuint FilterMin, FilterMax;

    Texture2D(const std::string& path, bool alpha = false, const TextureParams& params = {})
        : WrapS(params.wrapS), WrapT(params.wrapT),
          FilterMin(params.filterMin), FilterMax(params.filterMax)
    {
        glGenTextures(1, &ID);
        if (alpha)
        {
            InternalFormat = GL_RGBA;
            ImageFormat = GL_RGBA;
        }
        else
        {
            InternalFormat = GL_RGB;
            ImageFormat = GL_RGB;
        }

        // Load image
        unsigned char* image = loadImage(path.c_str());
        if (image)
        {
            generate(image);
            stbi_image_free(image); // Free after texture generation
        }
        else
        {
            std::cerr << "ERROR::TEXTURE2D: Failed to load texture: " << path << std::endl;
        }
    }

    void Bind() const
    {
        glBindTexture(GL_TEXTURE_2D, ID);
    }

private:
    // Helper function to load the image data
    unsigned char* loadImage(const char* path)
    {
        int width, height, channels;
        unsigned char* image = stbi_load(path, &width, &height, &channels, 0);
        if (image)
        {
            Width = width;
            Height = height;
        }
        return image;
    }

    void generate(unsigned char* data)
    {
        // Create Texture
        glBindTexture(GL_TEXTURE_2D, ID);
        glTexImage2D(GL_TEXTURE_2D, 0, InternalFormat, Width, Height, 0, ImageFormat, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
        // Set Texture wrap and filter modes
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, WrapS);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, WrapT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, FilterMin);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, FilterMax);
        // Unbind texture
        glBindTexture(GL_TEXTURE_2D, 0);
    }
};
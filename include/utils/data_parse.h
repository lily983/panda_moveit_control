#pragma once

#include <string>
#include <filesystem.h>

void CopyMeshFromWebToRepo(std::string web_path, std::string repo_path)
{
        if(std::filesystem::exists(repo_path)){
            std::filesystem::remove(repo_path);
        }
        std::filesystem::copy_file(web_path, repo_path);
}
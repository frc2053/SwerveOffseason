import os
import requests

# GitHub repository details
repo_owner = "mjansen4857"
repo_name = "pathplanner"
branch = "675-trajectory-generation-v2"

# Folders to download
folders = [
    "pathplannerlib/src/main/native/cpp",
    "pathplannerlib/src/main/native/include",
]

# Destination directories (change these to your project's subdirectories)
destinations = ["src/main/cpp", "src/main/include"]


def download_folder(owner, repo, path, branch, destination):
    url = f"https://api.github.com/repos/{owner}/{repo}/contents/{path}?ref={branch}"
    response = requests.get(url)
    if response.status_code == 200:
        contents = response.json()
        for item in contents:
            if item["type"] == "file":
                download_file(
                    item["download_url"], os.path.join(destination, item["name"])
                )
            elif item["type"] == "dir":
                subdir = os.path.join(destination, item["name"])
                os.makedirs(subdir, exist_ok=True)
                download_folder(owner, repo, item["path"], branch, subdir)
    else:
        print(f"Failed to fetch contents: {response.status_code}")


def download_file(url, destination):
    response = requests.get(url)
    if response.status_code == 200:
        os.makedirs(os.path.dirname(destination), exist_ok=True)
        with open(destination, "wb") as f:
            f.write(response.content)
        print(f"Downloaded: {destination}")
    else:
        print(f"Failed to download file: {url}")


# Download and copy folders
for folder, destination in zip(folders, destinations):
    print(f"Downloading {folder} to {destination}")
    download_folder(repo_owner, repo_name, folder, branch, destination)

print("Download and copy process completed.")

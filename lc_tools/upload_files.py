import requests
import os
import argparse

def connect(url, method, files=None, data=None):
    try:
        if method == "POST":
            files_dict = {
                'pcdFile': (os.path.basename(files["pcdFile"]), open(files["pcdFile"], "rb")),
                'imgFile': (os.path.basename(files["imgFile"]), open(files["imgFile"], "rb")),
            }

            # send request
            response = requests.post(url, files=files_dict)

            # close files
            for file in files_dict.values():
                file[1].close()

            # check response
            if response.status_code == 200:
                result = response.json()
                return result
            else:
                return {
                    "code": response.status_code,
                    "message": "Failed to connect to server.",
                    "data": False
                }
    except Exception as e:
        return {
            "code": 500,
            "msg": f"发生错误: {str(e)}",
            "data": False
        }

def main():
    # parse args
    parser = argparse.ArgumentParser(description="Lidar Curve Model Server")
    parser.add_argument("-p", "--pcd", type=str, help="PCD file path")
    parser.add_argument("-i", "--img", type=str, help="Image file path")
    args = parser.parse_args()
    
    url = "http://1.116.227.218:8894/api/pcd/file"
    method = "POST"
    files = {
        "pcdFile": args.pcd,
        "imgFile": args.img,
    }
    result = connect(url, method, files)
    print(result)

if __name__ == "__main__":
    main()
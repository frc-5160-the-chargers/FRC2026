import asyncio
import base64
import json

import aiohttp
import os
import re

from datetime import datetime

user_vendordeps = os.listdir("../vendordeps")
current_year = datetime.now().year
directory_regex = r"^20(\d{2})_?((a|A)lpha|(b|B)eta)?\d?$"
year_regex = r"^20(\d{2})"
github_api_token = os.getenv("github_api_token")
headers = {
    'Authorization': f'Bearer {github_api_token}' if github_api_token else '',
    'Accept': 'application/vnd.github+json'
}

async def update_dep(session: aiohttp.ClientSession, filename: str, url: str):
    async with session.get(url, headers=headers) as resp:
        content = base64.b64decode((await resp.json())["content"])
        json_content = json.loads(content.decode("utf-8"))
    with open("../vendordeps/" + filename, 'w') as f:
        f.write(json.dumps(json_content, indent=4))


async def main():

    async with aiohttp.ClientSession() as session:
        api_url = "https://api.github.com/repos/wpilibsuite/vendor-json-repo/contents"
        async with session.get(api_url, headers=headers) as resp:
            all_items = await resp.json()
            vendordep_sets = [
                item for item in all_items
                if re.match(directory_regex, item["name"])
                and float(re.match(year_regex, item["name"]).group()) >= current_year
            ]
        print("Pull from which set of vendordeps?")
        for i in range(len(vendordep_sets)):
            print(vendordep_sets[i]["name"] + "(" + str(i) + ")")
        choice = int(input("Input the set #: "))
        vendordep_set = vendordep_sets[choice]
        async with session.get(vendordep_set["url"], headers=headers) as resp:
            json_data = await resp.json()
            vendordep_options = {v["name"]: v["url"] for v in json_data if ".json" in v["name"]}
        tasks = []
        for vendordep in user_vendordeps:
            vendordep_name = vendordep[:vendordep.find(".json")]
            versions = [name for name in vendordep_options.keys() if vendordep_name in name]
            print(versions)
            if len(versions) == 0:
                continue
            tasks.append(update_dep(session, vendordep, vendordep_options[versions[-1]]))
        await asyncio.gather(*tasks)

if __name__ == '__main__':
    asyncio.run(main())

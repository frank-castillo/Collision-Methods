#pragma once



const unsigned int FS_NUM_VK=256;


class FsWin32KeyMapper
{
public:
	FsWin32KeyMapper();
	~FsWin32KeyMapper();

	int VkToFsKey(int vk);
	int FsKeyToVk(int fsKey);

protected:
	void AddKeyMapping(int fskey,int vk);

	int *mapVKtoFSKEY;
	int *mapFSKEYtoVK;
};


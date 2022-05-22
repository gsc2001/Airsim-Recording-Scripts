import airsim


def assignIds(client: airsim.MultirotorClient):
    _id = 1
    client.simSetSegmentationObjectID('floor', _id)
    _id += 1

    for i in range(293):
        meshName = 'a' + str(i)
        client.simSetSegmentationObjectID(meshName, _id)
        _id += 1

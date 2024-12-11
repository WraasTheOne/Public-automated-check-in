import * as jspb from 'google-protobuf'



export class StringRequest extends jspb.Message {
  getWord(): string;
  setWord(value: string): StringRequest;

  serializeBinary(): Uint8Array;
  toObject(includeInstance?: boolean): StringRequest.AsObject;
  static toObject(includeInstance: boolean, msg: StringRequest): StringRequest.AsObject;
  static serializeBinaryToWriter(message: StringRequest, writer: jspb.BinaryWriter): void;
  static deserializeBinary(bytes: Uint8Array): StringRequest;
  static deserializeBinaryFromReader(message: StringRequest, reader: jspb.BinaryReader): StringRequest;
}

export namespace StringRequest {
  export type AsObject = {
    word: string,
  }
}

export class StringResponse extends jspb.Message {
  getCharactersList(): Array<string>;
  setCharactersList(value: Array<string>): StringResponse;
  clearCharactersList(): StringResponse;
  addCharacters(value: string, index?: number): StringResponse;

  serializeBinary(): Uint8Array;
  toObject(includeInstance?: boolean): StringResponse.AsObject;
  static toObject(includeInstance: boolean, msg: StringResponse): StringResponse.AsObject;
  static serializeBinaryToWriter(message: StringResponse, writer: jspb.BinaryWriter): void;
  static deserializeBinary(bytes: Uint8Array): StringResponse;
  static deserializeBinaryFromReader(message: StringResponse, reader: jspb.BinaryReader): StringResponse;
}

export namespace StringResponse {
  export type AsObject = {
    charactersList: Array<string>,
  }
}

